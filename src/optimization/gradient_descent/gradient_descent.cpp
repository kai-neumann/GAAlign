//
// Created by Kai on 23.02.2022.
//

#include "gradient_descent.h"

#include <random>
#include <iterator>
#include <algorithm>
#include <chrono>

#include <geometry/motor_estimation.h>
#include <geometry/motor_estimation_sse.h>
#include <numeric>

#include <optimization/fast_shuffle.h>


// Custom fast random function
uint32_t x = 123456789;
uint32_t y = 362436069;
uint32_t z = 521288629;
uint32_t w = 88675123;
uint32_t xorshift128()
{
    uint32_t t = x ^ (x << 11);
    x = y; y = z; z = w;
    w ^= (w >> 19) ^ t ^ (t >> 8);
    return w;
}

gaalign::Motor gaalign::GradientDescentOptimizer::optimize(const std::vector<gaalign::Correspondence> &correspondences) const {
    if(m_settings.verbose) std::cout << "Running Gradient descent.." << std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Initialize the time it took to do only the motor estimation
    double timeMotorOnly = 0;
    double timeSamplingOnly = 0;
    double timeTransformOnly = 0;

    // Initialize the final transformation as identity
    gaalign::Motor combined = gaalign::Motor::identity();

    // Use a momentum term
    gaalign::Motor momentum;

    // Initialize runtime array
    std::vector<gaalign::TriangleCorrespondence> sampledTriangles(m_settings.trianglesPerIteration);

    // First generate the index array
    std::vector<std::uint32_t> indices;
    if(m_settings.precalculateIndices){
        // The array needs to be at least triangles*iterations long, and should also include all available correspondences
        indices.resize((int)fmax(m_settings.trianglesPerIteration*m_settings.maxIterations, correspondences.size()) + 2);
        std::iota(indices.begin(), indices.end(), 0); // Fill with values [0, 1, 2, ..]
        for(int i=correspondences.size(); i<indices.size(); i++) {
            indices[i] = indices[i] % correspondences.size();
        }
        shuffle_pcg_divisionless_with_slight_bias(indices.data(), indices.size());

        // Cut down if necessary
        if(correspondences.size() > m_settings.trianglesPerIteration*m_settings.maxIterations) {
            indices.resize(m_settings.trianglesPerIteration*m_settings.maxIterations);
        }
    }

    // Run the alignment iterations
    for (int iter = 0; iter < m_settings.maxIterations; iter++) {
        if(m_settings.verbose) std::cout << "Alignment iteration " << iter + 1 << "\n";

        //std::chrono::steady_clock::time_point beginSampling = std::chrono::steady_clock::now();

        // First sample n pairs of triangles and put them into a data structure (Takes ~0.01ms -> insignificant)
        #pragma omp parallel for
        for(int i=0; i<m_settings.trianglesPerIteration; i++) {
            // Add to output
            if(m_settings.precalculateIndices) {
                int offset = iter*m_settings.trianglesPerIteration + i;
                sampledTriangles[i].first.p1 = correspondences[indices[offset]].first;
                sampledTriangles[i].first.p2 = correspondences[indices[offset + 1]].first;
                sampledTriangles[i].first.p3 = correspondences[indices[offset + 2]].first;
                sampledTriangles[i].second.p1 = correspondences[indices[offset]].second;
                sampledTriangles[i].second.p2 = correspondences[indices[offset + 1]].second;
                sampledTriangles[i].second.p3 = correspondences[indices[offset + 2]].second;
            }
            else {
                // Sample three indices
                unsigned int idx1 = xorshift128() % correspondences.size();
                unsigned int idx2 = xorshift128() % correspondences.size();
                unsigned int idx3 = xorshift128() % correspondences.size();

                // Skip
                if(idx1 == idx2 || idx1 == idx3 || idx2 == idx3) {
                    continue;
                }

                sampledTriangles[i].first.p1 = correspondences[idx1].first;
                sampledTriangles[i].first.p2 = correspondences[idx2].first;
                sampledTriangles[i].first.p3 = correspondences[idx3].first;
                sampledTriangles[i].second.p1 = correspondences[idx1].second;
                sampledTriangles[i].second.p2 = correspondences[idx2].second;
                sampledTriangles[i].second.p3 = correspondences[idx3].second;

                //std::vector<int> sampled;
                //sampleSubIndices(correspondences.size(), 3, sampled);

                // Get the triangles
                /*sampledTriangles[i].first.p1 = correspondences[sampled[0]].first;
                sampledTriangles[i].first.p2 = correspondences[sampled[1]].first;
                sampledTriangles[i].first.p3 = correspondences[sampled[2]].first;
                sampledTriangles[i].second.p1 = correspondences[sampled[0]].second;
                sampledTriangles[i].second.p2 = correspondences[sampled[1]].second;
                sampledTriangles[i].second.p3 = correspondences[sampled[2]].second;*/
            }

        }

        //auto endSampling = std::chrono::high_resolution_clock::now();
        //double samplingTimeMS = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(endSampling - beginSampling).count())/1000000.0;
        //timeSamplingOnly += samplingTimeMS;

        //std::chrono::steady_clock::time_point beginTransform = std::chrono::steady_clock::now();

        // Transform all source triangles with the combined motor of all previous iterations (This takes ~0.8ms for all iterations)
        #pragma omp parallel for
        for(int i=0; i<sampledTriangles.size(); i++) {
            sampledTriangles[i].first.applyMotor(combined);
        }

        /*auto endTransform = std::chrono::high_resolution_clock::now();
        double transformTimeMS = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(endTransform - beginTransform).count())/1000000.0;
        timeTransformOnly += transformTimeMS;*/

        // Start timer
        std::chrono::steady_clock::time_point beginMotor = std::chrono::steady_clock::now();

        // Then calculate a motor per set of triangles and average it
        gaalign::Motor avgMotor;
        #pragma omp parallel for shared(avgMotor)
        for(int i=0; i<sampledTriangles.size(); i++) {
            // Convert to format necessary for current method call
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> tri;
            tri.emplace_back(sampledTriangles[i].first.p1, sampledTriangles[i].second.p1);
            tri.emplace_back(sampledTriangles[i].first.p2, sampledTriangles[i].second.p2);
            tri.emplace_back(sampledTriangles[i].first.p3, sampledTriangles[i].second.p3);

            gaalign::Motor motor;
            // If SSE3 is enabled in the settings and it is suppored: Try to use it!
            if(m_settings.enableSSE3) {
                #ifdef __SSE3__
                    // Estimate Motor using SSE3 if it is available
                    estimateMotorFromThreeCorrespondencesSSE3(tri, motor);
                #else
                    std::cout << "WARINING: SSE3 was enabled in the settings, but is not supported by this platform!" << std::endl;
                    // Else use standard C code for motor estimation
                    estimateMotorFromThreeCorrespondences(tri, motor);
                #endif
            }
            // Else: Use default C++ version
            else {
                estimateMotorFromThreeCorrespondences(tri, motor);
            }


            // If the motor was not finite: Do not add it to the average
            if(!std::isfinite(motor.data[0])) {
                continue;
            }

            // Add to average motor
            #pragma omp critical
            {
                // Add to average
                avgMotor += motor;
            };
        }

        // End timer
        auto endMotor = std::chrono::high_resolution_clock::now();
        timeMotorOnly += ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(endMotor - beginMotor).count())/1000000.0;

        if(m_settings.verbose) avgMotor.print();

        // Scale the motor by the current step size (it needs to be normalized first)
        if(iter != 0) {
            avgMotor.normalize();
            avgMotor.scale(m_settings.stepSize);
        }


        // Apply momentum
        if(m_settings.useMomentum) {
            // Scale the last momentum
            momentum.scale(m_settings.momentumStrength);

            // Add the momentum (if wished)
            for (int i = 0; i < avgMotor.data.size(); i++) {
                momentum.data[i] = avgMotor.data[i] + momentum.data[i];
                avgMotor.data[i] = momentum.data[i];
            }
        }

        // Join new motor with the combined motor
        combined = gaalign::Motor::join(combined, avgMotor);
    }

    // End timer
    auto end = std::chrono::high_resolution_clock::now();
    double timeMS = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/1000000.0;
    if(m_settings.printTiming) std::cout << "Finished Optimization in " << timeMS << " ms (Triangle Interpolation took " << timeMotorOnly << " ms)" << std::endl;
    //if(m_settings.printTiming) std::cout << "Sampling took " << timeSamplingOnly << " ms" << std::endl;
    //if(m_settings.printTiming) std::cout << "Transforming took " << timeTransformOnly << " ms (" << timeTransformOnly / ((double)(3*m_settings.trianglesPerIteration*m_settings.maxIterations)) << " per point)" << std::endl;

    return combined;
}

std::string gaalign::GradientDescentOptimizer::getName() const {
    return "Gradient Descent";
}

gaalign::GradientDescentSettings &gaalign::GradientDescentOptimizer::getSettings() {
    return m_settings;
}
