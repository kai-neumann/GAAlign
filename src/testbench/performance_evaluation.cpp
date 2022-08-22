//
// Created by Kai on 01.05.2022.
//

#include "performance_evaluation.h"

#include <random>
#include <fstream>

#include <testbench/wrappers_optim/optimization_wrapper.h>
#include <testbench/wrappers_optim/gaalign_optimization_wrapper.h>
#include <testbench/wrappers_optim/gaalign_cuda_optimization_wrapper.h>
#include <testbench/wrappers_optim/pcl_optimization_wrapper.h>
#include <testbench/wrappers/pcl_icp_wrapper.h>
#include <testbench/wrappers_optim/ga_lms_optimization_wrapper.h>


#include <optimization/fast_shuffle.h>

// Matplotlib
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;


void compareRuntimePerformance(const std::string &resourceFolder, bool visualize) {
    // First define a list of all models that should be used for testing
    std::vector<std::string> modelNames = {"Suzanne.ply", "Bunny.ply", "Dragon.ply", "Ship.ply"};

    // Load all models
    std::cout << "Loading models.." << std::endl;
    std::vector<gaalign::PointCloud> models;
    models.reserve(modelNames.size());
    for(const auto& name : modelNames) {
        models.emplace_back(resourceFolder + "/models/" + name);
    }

    // Initialize the algorithms
    std::vector<std::shared_ptr<gaalign::OptimizationAlgorithmWrapper>> algorithms;
    /*algorithms.push_back(std::make_shared<gaalign::GaalignOptimizationWrapper>(false, false));
    algorithms.push_back(std::make_shared<gaalign::PCLICPOptimizationWrapper>(gaalign::PCL_ICP_VARIANT::SVD_BASED));
    algorithms.push_back(std::make_shared<gaalign::PCLICPOptimizationWrapper>(gaalign::PCL_ICP_VARIANT::POINT_TO_PLANE));
    algorithms.push_back(std::make_shared<gaalign::PCLICPOptimizationWrapper>(gaalign::PCL_ICP_VARIANT::SYMMETRIC_POINT_TO_PLANE));
    algorithms.push_back(std::make_shared<gaalign::GA_LMS_OptimizationWrapper>(false));*/
    algorithms.push_back(std::make_shared<gaalign::GA_LMS_OptimizationWrapper>(true));

    // Parameters
    int repetitions = 100;
    int minCorrespondences = 100;
    int maxCorrespondences = 10000;
    int steps = 3;

    // Calculate the step size based on the number of steps and the interval
    double stepSize = (maxCorrespondences - minCorrespondences)/((double)steps-1);

    // Settings for the random transform
    double maxRelativeTranslation = 0.2;
    double maxRotationDegree = 20;
    double minScaleFactor = 1;
    double maxScaleFactor = 1;

    // Initialize the constant random distributions
    srand((std::random_device())());
    std::mt19937 re((std::random_device())());
    std::uniform_real_distribution<double> overlapDistribution(0.2, 1);

    // Init the result arrays
    std::vector<std::vector<double>> runtimes;
    std::vector<double> correspondenceCounts;
    for(int i=0; i<algorithms.size(); i++) {
        runtimes.emplace_back();
    }

    // Use increasing noise levels
    for(int iter=0; iter<steps; iter++) {
        // Calculate the current number of correspondences
        int iterationCorrespondenceCount = minCorrespondences + iter*stepSize;

        // Init the temp arrays
        std::vector<std::vector<double>> tempRuntimes;
        for(int i=0; i<algorithms.size(); i++) {
            tempRuntimes.emplace_back();
        }

        // Loop over all models
        for(const auto& model : models) {
            // Get the maximum dimension of the original point cloud, to calculate the noise scale
            double maxDimension = model.getMaxDimension();

            // Initialize model specific distribution
            std::uniform_real_distribution<double> translationDist(-maxRelativeTranslation * maxDimension, maxRelativeTranslation * maxDimension);
            std::uniform_real_distribution<double> angleDist(-maxRotationDegree * 0.01745329, maxRotationDegree * 0.01745329);
            std::uniform_real_distribution<double> sizeDist(minScaleFactor, maxScaleFactor);

            // Do N repetitions
            for(int repetition=0; repetition<repetitions; repetition++) {
                std::cout << "Correspondences: " << iterationCorrespondenceCount << "; Model: " << model.getName() << "; Repetition: " << repetition << std::endl;

                // Apply a random transformation to the model
                gaalign::PointCloud transformed = model;
                {
                    Eigen::AngleAxisd rollAngle(angleDist(re), Eigen::Vector3d::UnitZ());
                    Eigen::AngleAxisd yawAngle(angleDist(re), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd pitchAngle(angleDist(re), Eigen::Vector3d::UnitX());
                    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
                    Eigen::Matrix3d rotationMatrix = q.matrix();
                    double scaling = sizeDist(re);

                    // Random translation
                    Eigen::Vector3d translation(translationDist(re), translationDist(re), translationDist(re));

                    // Apply random rotation and translation to the source
                    for(int i=0; i<transformed.size(); i++) {
                        transformed.setPoint(i, scaling * rotationMatrix * model.getPoint(i) + translation);
                        transformed.setNormal(i, rotationMatrix * model.getNormal(i));
                    }
                }

                // Choose a number of random point pairs as correspondences
                //int correspondenceCount = correspondenceCountDist(re);
                int correspondenceCount = fmin(iterationCorrespondenceCount, model.size());

                // Initialize an array of values and shuffle it
                std::vector<std::size_t> indices(model.size());
                iota(indices.begin(), indices.end(), 0); // Fill with values [0, 1, 2, ..]
                std::shuffle(indices.begin(), indices.end(), re);

                // Use the first N values to build correspondences
                std::vector<gaalign::Correspondence> correspondences(correspondenceCount);
                std::vector<gaalign::Correspondence> correspondingNormals(correspondenceCount);
                #pragma omp parallel for
                for(int i=0; i<correspondenceCount; i++) {
                    correspondences[i] = gaalign::Correspondence(transformed.getPoint(indices[i]), model.getPoint(indices[i]));
                    correspondingNormals[i] = gaalign::Correspondence(transformed.getNormal(indices[i]), model.getNormal(indices[i]));
                }

                // For all algorithms
                for(int algorithmID=0; algorithmID<algorithms.size(); algorithmID++) {
                    // Run the algorithm
                    std::pair<Eigen::Matrix4d, double> result = algorithms[algorithmID]->calculateRegistration(correspondences, correspondingNormals);

                    // Add the rmse and runtime to the output arrays
                    tempRuntimes[algorithmID].push_back(result.second);
                }
            }
        }

        // Average the runtime and temp error per model for this noise level
        correspondenceCounts.push_back(iterationCorrespondenceCount);
        for(int i=0; i<algorithms.size(); i++) {
            double avgRuntime = avg(tempRuntimes[i]);

            // Add to output
            runtimes[i].push_back(avgRuntime);
        }
    }

    // Save results to csv
    char buff[96];
    {
        time_t now = time(NULL);
        strftime(buff, 96, "%Y-%m-%d_%H-%M-%S_runtime", localtime(&now));

        // FIXME: This only works if the build directory is directly beneath the project root folder
        std::string csvPath = resourceFolder + "/../../results/" + buff + ".csv";

        std::cout << "Saving to '" << buff << "'" << std::endl;

        std::ofstream csvfile;
        csvfile.open(csvPath);

        // First write the noise values
        csvfile << "All" << ";" << "Correspondences" << ";";
        for (int i = 0; i < correspondenceCounts.size(); i++) {
            if (i != 0) csvfile << ";";
            csvfile << correspondenceCounts[i];
        }
        csvfile << std::endl;

        for (int algorithmID = 0; algorithmID < algorithms.size(); algorithmID++) {
            // RUNTIME
            csvfile << algorithms[algorithmID]->getName() << ";" << "Runtime [ms]" << ";";
            for (int i = 0; i < runtimes[algorithmID].size(); i++) {
                if (i != 0) csvfile << ";";
                csvfile << runtimes[algorithmID][i];
            }
            csvfile << std::endl;
        }
        csvfile.close();
    }

    // Plot the different algorithms
    plt::figure_size(1200, 1000);

    plt::title("Runtime for different numbers of correspondences");
    plt::xlabel("Number of correspondences");
    plt::ylabel("Runtime [ms]");

    double maxRuntime = 0;
    for(int algorithmID=0; algorithmID < algorithms.size(); algorithmID++) {
        plt::named_plot(algorithms[algorithmID]->getName(), correspondenceCounts, runtimes[algorithmID]);

        for(const auto& runtime : runtimes[algorithmID]) {
            if(runtime > maxRuntime) {
                maxRuntime = runtime;
            }
        }
    }

    // Set the limits
    plt::ylim<double>(0, maxRuntime);
    plt::xlim<double>(0, maxCorrespondences);

    // Add the legend
    plt::legend();

    // Save to file
    plt::save(resourceFolder + "/../../results/" + buff + ".png");

    // Show the plot
    if(visualize) {
        plt::show();
    }

}
