//
// Created by Kai on 02.05.2022.
//

#include "hyperparameter_evaluation.h"
#include <iostream>
#include <fstream>
#include <geometry/point_cloud.h>
#include <testbench/wrappers_optim/gaalign_optimization_wrapper.h>
#include <optimization/gradient_descent/gradient_descent.h>
#include <random>
#include "testbench/common.h"
#include <chrono>

// Matplotlib
#include "matplotlibcpp.h"
#include "gradient_descent_hyperparameter_optimization.h"

namespace plt = matplotlibcpp;

void gaalign::evaluateInfluenceOfHyperparameters(const std::string &resourceFolder) {
    // First define a list of all models that should be used for testing
    std::vector<std::string> modelNames = {"Suzanne.ply", "Bunny.ply", "Dragon.ply", "Ship.ply"};

    // Load all models
    std::cout << "Loading models.." << std::endl;
    std::vector<gaalign::PointCloud> models;
    models.reserve(modelNames.size());
    for(const auto& name : modelNames) {
        models.emplace_back(resourceFolder + "/models/" + name);
    }

    // Downsample all models to at maximum 100k points -> For faster evaluation with nearly the same information value
    #pragma omp parallel for
    for(int i=0; i<models.size(); i++) {
        models[i] = models[i].subsample(100000);
    }

    //std::vector<double> noiseLevels = {0, 0.025, 0.05};
    std::vector<double> noiseLevels = {0.05};
    //std::vector<double> hyperparamTypes = {Hyperparameter::Iterations, Hyperparameter::StepSize, Hyperparameter::Triangles};
    std::vector<double> hyperparamTypes = {Hyperparameter::StepSize};

    // Run the three different types
    std::vector<HyperparameterResults> results;

    for(const auto& noiseLevel : noiseLevels) {
        //results.push_back(evaluateHyperparameter(Hyperparameter::Iterations, 5, 100, 25, noiseLevel, models));
        results.push_back(evaluateHyperparameter(Hyperparameter::StepSize, 0.01, 1.0, 10, noiseLevel, models));
        //results.push_back(evaluateHyperparameter(Hyperparameter::Triangles, 16, 2048, 25, noiseLevel, models));
    }


    // Plot the different algorithms
    plt::figure_size(1200, 1000);

    plt::suptitle("Influence of Hyperparameters on the Runtime and Accuracy");

    // Get the maximum runtime over all plots
    double maxRuntime = 0;
    for(const auto& res : results) {
        for(const auto time : res.runtimeValues) {
            if(time > maxRuntime) maxRuntime = time;
        }
    }

    // Get the maximum rmse over all plots
    double maxRMSE = 0;
    for(const auto& res : results) {
        for(const auto rmse : res.rmseValues) {
            if(rmse > maxRMSE) maxRMSE = rmse;
        }
    }

    // Save results to csv
    {
        char buff[96];
        time_t now = time(NULL);
        strftime(buff, 96, "%Y-%m-%d_%H-%M-%S_hyperparameters.csv", localtime(&now));

        // FIXME: This only works if the build directory is directly beneath the project root folder
        std::string csvPath = resourceFolder + "/../../results/" + buff;

        std::cout << "Saving to '" << buff << "'" << std::endl;

        std::ofstream csvfile;
        csvfile.open(csvPath);

        // Loop over all results
        for(const auto& result : results) {
            // Get the name of the result type
            std::string name = "";
            if(result.paramType == gaalign::Hyperparameter::Iterations) {
                name = "Iterations";
            }
            else if(result.paramType == gaalign::Hyperparameter::StepSize) {
                name = "Step Size";
            }
            else {
                name = "Triangles";
            }

            // First write out the sampled positions
            csvfile << name << ";noise=" << result.noiseLevel << ";paramValues" << ";";
            for (int i = 0; i < result.paramValues.size(); i++) {
                if (i != 0) csvfile << ";";
                csvfile << result.paramValues[i];
            }
            csvfile << std::endl;

            // Then write out the runtimes
            csvfile << name << ";noise=" << result.noiseLevel << ";Runtime [ms]" << ";";
            for (int i = 0; i < result.runtimeValues.size(); i++) {
                if (i != 0) csvfile << ";";
                csvfile << result.runtimeValues[i];
            }
            csvfile << std::endl;

            // Then write out the rmses
            csvfile << name << ";noise=" << result.noiseLevel << ";RMSE" << ";";
            for (int i = 0; i < result.rmseValues.size(); i++) {
                if (i != 0) csvfile << ";";
                csvfile << result.rmseValues[i];
            }
            csvfile << std::endl;
        }
    }

    // Loop over the results
    for(int i=0; i<hyperparamTypes.size(); i++) {

        // Plot the runtime
        plt::subplot(2, 3, i+1);
        {
            plt::title("Runtime");
            if(results[i].paramType == Iterations) {
                plt::xlabel("Number of iterations");
            }
            else if(results[i].paramType == StepSize) {
                plt::xlabel("Step size");
            }
            else {
                plt::xlabel("Number of Triangles");
            }

            plt::ylabel("Runtime [ms]");

            // Loop over all results and plot the ones with the correct type
            for(const auto& result : results) {
                if(result.paramType == hyperparamTypes[i]) {
                    plt::named_plot("σ=" + std::to_string(result.noiseLevel), result.paramValues, result.runtimeValues);
                }
            }

            // Set a global y limit
            plt::ylim(0.0, maxRuntime);

            plt::legend();
        }

        // Plot the rmse
        plt::subplot(2, 3, i+4);
        {
            plt::title("RMSE");
            if(results[i].paramType == Iterations) {

                plt::xlabel("Number of iterations");
            }
            else if(results[i].paramType == StepSize) {
                plt::xlabel("Step size");
            }
            else {
                plt::xlabel("Number of Triangles");
            }


            plt::ylabel("RMSE");

            // Loop over all results and plot the ones with the correct type
            for(const auto& result : results) {
                if(result.paramType == hyperparamTypes[i]) {
                    plt::named_plot("σ=" + std::to_string(result.noiseLevel), result.paramValues, result.rmseValues);
                }
            }

            // Set a global y limit
            plt::ylim(0.0, maxRMSE);

            plt::legend();
        }
    }



    // Show the plot
    plt::show();
}

gaalign::HyperparameterResults gaalign::evaluateHyperparameter(gaalign::Hyperparameter param, double min, double max, int steps,  double noiseStrength, std::vector<gaalign::PointCloud>& models) {

    // Initialize the algorithm
    GradientDescentOptimizer optim;
    optim.getSettings().precalculateIndices = false;
    optim.getSettings().enableSSE3 = false;

    // Parameters
    int repetitions = 2000;

    // Calculate the step size based on the number of steps and the interval
    double stepSize = (max - min)/((double)steps-1);

    // Settings for the random transform
    double maxRelativeTranslation = 0.1;
    double maxRotationDegree = 10;
    double minScaleFactor = 1;
    double maxScaleFactor = 1;

    // Initialize the constant random distributions
    srand((std::random_device())());
    std::mt19937 re((std::random_device())());
    std::uniform_real_distribution<double> overlapDistribution(0.2, 1);

    // Init the result
    HyperparameterResults results;
    results.paramType = param;
    results.noiseLevel = noiseStrength;

    // Use increasing noise levels
    for(int iter=0; iter<steps; iter++) {
        // Calculate the current number of correspondences
        double parameterValue = min + iter*stepSize;

        // Apply to optimizer
        if(param == Iterations) {
            optim.getSettings().maxIterations = (int)parameterValue;
        }
        else if(param == StepSize) {
            optim.getSettings().stepSize = parameterValue;
        }
        else {
            optim.getSettings().trianglesPerIteration = (int)parameterValue;
        }

        // Init the temp arrays
        std::vector<double> tempRuntimes;
        std::vector<double> tempRMSES;

        // Loop over all models
        for(const auto& model : models) {
            // Get the maximum dimension of the original point cloud, to calculate the noise scale
            double maxDimension = model.getMaxDimension();

            // Initialize model specific distribution
            std::uniform_real_distribution<double> translationDist(-maxRelativeTranslation * maxDimension, maxRelativeTranslation * maxDimension);
            std::uniform_real_distribution<double> angleDist(-maxRotationDegree * 0.01745329, maxRotationDegree * 0.01745329);
            std::uniform_real_distribution<double> sizeDist(minScaleFactor, maxScaleFactor);

            // Initialize the random distribution for this noise level and model
            std::normal_distribution<double> noiseDistribution(0, noiseStrength*maxDimension);

            // Do N repetitions
            for(int repetition=0; repetition<repetitions; repetition++) {
                if(param == Iterations) std::cout << "Iterations: " << (int)parameterValue << "; Model: " << model.getName() << "; Repetition: " << repetition << std::endl;
                if(param == StepSize) std::cout << "stepSize: " << parameterValue << "; Model: " << model.getName() << "; Repetition: " << repetition << std::endl;
                if(param == Triangles) std::cout << "Triangles: " << (int)parameterValue << "; Model: " << model.getName() << "; Repetition: " << repetition << std::endl;

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
                int correspondenceCount = 10000;

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

                // Add noise to the correspondences (not the model!)
                #pragma omp parallel for
                for(int i=0; i<correspondenceCount; i++) {
                    correspondences[i].first += Eigen::Vector3d(noiseDistribution(re), noiseDistribution(re), noiseDistribution(re));
                    correspondences[i].second += Eigen::Vector3d(noiseDistribution(re), noiseDistribution(re), noiseDistribution(re));
                    correspondingNormals[i].first = (correspondingNormals[i].first + Eigen::Vector3d(noiseDistribution(re), noiseDistribution(re), noiseDistribution(re))).normalized();
                    correspondingNormals[i].second = (correspondingNormals[i].second + Eigen::Vector3d(noiseDistribution(re), noiseDistribution(re), noiseDistribution(re))).normalized();
                }

                // Start the time measurement
                std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

                // Run the algorithm
                gaalign::Motor resultingMotor = optim.optimize(correspondences);

                // Stop the time measurement
                auto end = std::chrono::high_resolution_clock::now();
                double timeMS = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/1000000.0;

                // Apply the transformation to the point cloud
                gaalign::PointCloud aligned = transformed;
                aligned.applyMotor(resultingMotor);

                // Measure the rmse between the aligned model and the original model
                double squaredErrorSum = 0;
                for(int i=0; i<aligned.size(); i++) {
                    squaredErrorSum += (aligned.getPoint(i) - model.getPoint(i)).squaredNorm();
                }
                double rmse = sqrt(squaredErrorSum / (double)transformed.size()) / maxDimension;

                // Add the rmse and runtime to the output arrays
                tempRuntimes.push_back(timeMS);
                tempRMSES.push_back(rmse);
            }
        }

        // Average the runtime and temp error per model for this noise level
        results.paramValues.push_back(parameterValue);
        double avgRuntime = avg(tempRuntimes);
        double avgRMSE = avg(tempRMSES);

        // Add to output
        results.rmseValues.push_back(avgRMSE);
        results.runtimeValues.push_back(avgRuntime);
    }


    return results;
}
