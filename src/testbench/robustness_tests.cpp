//
// Created by Kai on 27.04.2022.
//

#include "robustness_tests.h"

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

double avgVec(const std::vector<double>& vec) {
    double sum = 0;
    for(double i : vec) {
        sum += i;
    }
    return sum / ((double)vec.size());
}

void compareRobustnessAgainstNoise(const std::string &resourceFolder, bool visualize) {
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

    // Initialize the algorithms
    std::vector<std::shared_ptr<gaalign::OptimizationAlgorithmWrapper>> algorithms;
    algorithms.push_back(std::make_shared<gaalign::GaalignOptimizationWrapper>(false, false));
    //algorithms.push_back(std::make_shared<gaalign::GaalignOptimizationWrapper>(true, false));
    //algorithms.push_back(std::make_shared<gaalign::GaalignOptimizationWrapper>(false, true));
    //algorithms.push_back(std::make_shared<gaalign::GaalignOptimizationWrapper>(true, true));
    //algorithms.push_back(std::make_shared<gaalign::GaalignCudaOptimizationWrapper>());
    algorithms.push_back(std::make_shared<gaalign::PCLICPOptimizationWrapper>(gaalign::PCL_ICP_VARIANT::SVD_BASED));
    algorithms.push_back(std::make_shared<gaalign::PCLICPOptimizationWrapper>(gaalign::PCL_ICP_VARIANT::POINT_TO_PLANE));
    algorithms.push_back(std::make_shared<gaalign::PCLICPOptimizationWrapper>(gaalign::PCL_ICP_VARIANT::SYMMETRIC_POINT_TO_PLANE));
    algorithms.push_back(std::make_shared<gaalign::GA_LMS_OptimizationWrapper>(false));
    algorithms.push_back(std::make_shared<gaalign::GA_LMS_OptimizationWrapper>(true));

    // Parameters
    int repetitions = 200;
    double minNoise = 0;
    double maxNoise = 0.2;
    double stepSize = 0.01;

    // Settings for the random transform
    double maxRelativeTranslation = 0.2;
    double maxRotationDegree = 20;
    double minScaleFactor = 1;
    double maxScaleFactor = 1;

    // Initialize the constant random distributions
    srand((std::random_device())());
    std::mt19937 re((std::random_device())());
    std::uniform_real_distribution<double> overlapDistribution(0.2, 1);
    std::uniform_int_distribution<int> correspondenceCountDist(100, 2000);

    // Init the result arrays
    std::vector<std::vector<double>> rmses;
    std::vector<std::vector<double>> runtimes;
    std::vector<double> noiseLevels;
    for(int i=0; i<algorithms.size(); i++) {
        rmses.emplace_back();
        runtimes.emplace_back();
    }

    // Use increasing noise levels
    for(double noiseLevel=minNoise; noiseLevel<=maxNoise; noiseLevel+=stepSize) {
        // Init the temp arrays
        std::vector<std::vector<double>> tempRuntimes;
        std::vector<std::vector<double>> tempErrors;
        for(int i=0; i<algorithms.size(); i++) {
            tempRuntimes.emplace_back();
            tempErrors.emplace_back();
        }

        // Loop over all models
        for(const auto& model : models) {
            // Get the maximum dimension of the original point cloud, to calculate the noise scale
            double maxDimension = model.getMaxDimension();

            // Initialize model specific distribution
            std::uniform_real_distribution<double> translationDist(-maxRelativeTranslation * maxDimension, maxRelativeTranslation * maxDimension);
            std::uniform_real_distribution<double> angleDist(-maxRotationDegree * 0.01745329, maxRotationDegree * 0.01745329);
            std::uniform_real_distribution<double> sizeDist(minScaleFactor, maxScaleFactor);

            // Initialize the random distribution for this noise level and model
            std::normal_distribution<double> noiseDistribution(0, noiseLevel*maxDimension);

            // Do N repetitions
            for(int repetition=0; repetition<repetitions; repetition++) {
                std::cout << "Noise level: " << noiseLevel << "; Model: " << model.getName() << "; Repetition: " << repetition << std::endl;

                // Apply a random transformation to the model
                gaalign::PointCloud transformed = model;
                {
                    Eigen::AngleAxisd rollAngle(angleDist(re), Eigen::Vector3d::UnitZ());
                    Eigen::AngleAxisd yawAngle(angleDist(re), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd pitchAngle(angleDist(re), Eigen::Vector3d::UnitX());
                    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
                    Eigen::Matrix3d rotationMatrix = q.matrix();
                    double scaling = sizeDist(re);

                    // Print the inverse
                    //std::cout << "GOAL: " << rotationMatrix.inverse() << std::endl;

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
                int correspondenceCount = 1000;

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

                // For all algorithms
                for(int algorithmID=0; algorithmID<algorithms.size(); algorithmID++) {
                    // Run the algorithm
                    std::pair<Eigen::Matrix4d, double> result = algorithms[algorithmID]->calculateRegistration(correspondences, correspondingNormals);

                    // Apply the transformation to the point cloud
                    gaalign::PointCloud aligned = transformed;
                    #pragma omp parallel for
                    for(int i=0; i<transformed.size(); i++) {
                        // Apply transformation
                        Eigen::Vector4d point = result.first * Eigen::Vector4d(transformed.getPoint(i).x(), transformed.getPoint(i).y(), transformed.getPoint(i).z(), 1);
                        Eigen::Vector4d normal = result.first * Eigen::Vector4d(transformed.getNormal(i).x(), transformed.getNormal(i).y(), transformed.getNormal(i).z(), 1);

                        // Set transformed point / normal to point cloud
                        aligned.setPoint(i, Eigen::Vector3d(point.x(), point.y(), point.z()));
                        aligned.setNormal(i, Eigen::Vector3d(normal.x(), normal.y(), normal.z()));
                    }

                    // Measure the rmse between the aligned model and the original model
                    double squaredErrorSum = 0;
                    for(int i=0; i<aligned.size(); i++) {
                        squaredErrorSum += (aligned.getPoint(i) - model.getPoint(i)).squaredNorm();
                    }
                    double rmse = sqrt(squaredErrorSum / (double)transformed.size()) / maxDimension;

                    if(isnan(rmse)) {
                        std::cout << "Encountered a nan error for " << algorithms[algorithmID]->getName() << std::endl;
                        std::cout << result.first << std::endl;
                    }
                    else {
                        // Add the rmse and runtime to the output arrays
                        tempRuntimes[algorithmID].push_back(result.second);
                        tempErrors[algorithmID].push_back(rmse);
                    }
                }
            }
        }

        // Average the runtime and temp error per model for this noise level
        noiseLevels.push_back(noiseLevel);
        for(int i=0; i<algorithms.size(); i++) {
            double avgRuntime = avgVec(tempRuntimes[i]);
            double avgError = avgVec(tempErrors[i]);

            // Add to output
            rmses[i].push_back(avgError);
            runtimes[i].push_back(avgRuntime);
        }
    }

    // Save results to csv
    char buff[96];
    {
        time_t now = time(NULL);
        strftime(buff, 96, "%Y-%m-%d_%H-%M-%S_robustness_noise", localtime(&now));

        // FIXME: This only works if the build directory is directly beneath the project root folder
        std::string csvPath = resourceFolder + "/../../results/" + buff + ".csv";

        std::cout << "Saving to '" << buff << "'" << std::endl;

        std::ofstream csvfile;
        csvfile.open(csvPath);

        // First write the noise values
        csvfile << "All" << ";" << "Noise" << ";";
        for (int i = 0; i < noiseLevels.size(); i++) {
            if (i != 0) csvfile << ";";
            csvfile << noiseLevels[i];
        }
        csvfile << std::endl;

        for (int algorithmID = 0; algorithmID < algorithms.size(); algorithmID++) {
            // RMSE
            csvfile << algorithms[algorithmID]->getName() << ";" << "RMSE" << ";";
            for (int i = 0; i < rmses[algorithmID].size(); i++) {
                if (i != 0) csvfile << ";";
                csvfile << rmses[algorithmID][i];
            }
            csvfile << std::endl;

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
    plt::figure_size(1200, 600);

    plt::subplot(1, 2, 1);
    plt::title("RMSE over noise strength");
    plt::xlabel("Noise strength");
    plt::ylabel("RMSE");

    for(int algorithmID=0; algorithmID < algorithms.size(); algorithmID++) {
        plt::named_plot(algorithms[algorithmID]->getName(), noiseLevels, rmses[algorithmID]);
    }

    // Add the legend
    plt::legend();

    // Plot the runtimes
    plt::subplot(1, 2, 2);
    plt::title("Runtime over noise strength");
    plt::xlabel("Noise strength");
    plt::ylabel("Runtime [ms]");

    double maxRuntime = 0;
    for(int algorithmID=0; algorithmID < algorithms.size(); algorithmID++) {
        plt::named_plot(algorithms[algorithmID]->getName(), noiseLevels, runtimes[algorithmID]);

        for(const auto& runtime : runtimes[algorithmID]) {
            if(runtime > maxRuntime) {
                maxRuntime = runtime;
            }
        }
    }

    // Set the limits
    plt::ylim<double>(0, maxRuntime);

    // Add the legend
    plt::legend();

    // Save to file
    plt::save(resourceFolder + "/../../results/" + buff + ".png");

    // Show the plot
    if(visualize) {
        plt::show();
    }

}

void compareRobustnessAgainstOutliers(const std::string &resourceFolder, bool visualize) {
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

    // Initialize the algorithms
    std::vector<std::shared_ptr<gaalign::OptimizationAlgorithmWrapper>> algorithms;
    algorithms.push_back(std::make_shared<gaalign::GaalignOptimizationWrapper>(false, false));
    //algorithms.push_back(std::make_shared<gaalign::GaalignOptimizationWrapper>(true, false));
    //algorithms.push_back(std::make_shared<gaalign::GaalignOptimizationWrapper>(false, true));
    //algorithms.push_back(std::make_shared<gaalign::GaalignOptimizationWrapper>(true, true));
    //algorithms.push_back(std::make_shared<gaalign::GaalignCudaOptimizationWrapper>());
    algorithms.push_back(std::make_shared<gaalign::PCLICPOptimizationWrapper>(gaalign::PCL_ICP_VARIANT::SVD_BASED));
    algorithms.push_back(std::make_shared<gaalign::PCLICPOptimizationWrapper>(gaalign::PCL_ICP_VARIANT::POINT_TO_PLANE));
    algorithms.push_back(std::make_shared<gaalign::PCLICPOptimizationWrapper>(gaalign::PCL_ICP_VARIANT::SYMMETRIC_POINT_TO_PLANE));
    algorithms.push_back(std::make_shared<gaalign::GA_LMS_OptimizationWrapper>(false));
    algorithms.push_back(std::make_shared<gaalign::GA_LMS_OptimizationWrapper>(true));

    // Parameters
    int repetitions = 200;
    double minOutlierPercentage = 0;
    double maxOutlierPercentage = 0.5;
    double stepSize = 0.02;

    // Settings for the random transform
    double maxRelativeTranslation = 0.2;
    double maxRotationDegree = 20;
    double minScaleFactor = 1;
    double maxScaleFactor = 1;

    // Initialize the constant random distributions
    srand((std::random_device())());
    std::mt19937 re((std::random_device())());
    std::uniform_real_distribution<double> overlapDistribution(0.2, 1);
    std::uniform_int_distribution<int> correspondenceCountDist(100, 2000);

    // Init the result arrays
    std::vector<std::vector<double>> rmses;
    std::vector<std::vector<double>> runtimes;
    std::vector<double> outlierPercentages;
    for(int i=0; i<algorithms.size(); i++) {
        rmses.emplace_back();
        runtimes.emplace_back();
    }

    // Use increasing noise levels
    for(double outlierPercentage=minOutlierPercentage; outlierPercentage<=maxOutlierPercentage; outlierPercentage+=stepSize) {
        // Init the temp arrays
        std::vector<std::vector<double>> tempRuntimes;
        std::vector<std::vector<double>> tempErrors;
        for(int i=0; i<algorithms.size(); i++) {
            tempRuntimes.emplace_back();
            tempErrors.emplace_back();
        }

        // Loop over all models
        for(const auto& model : models) {
            // Get the maximum dimension of the original point cloud, to calculate the noise scale
            double maxDimension = model.getMaxDimension();

            // Initialize model specific distribution
            std::uniform_real_distribution<double> translationDist(-maxRelativeTranslation * maxDimension, maxRelativeTranslation * maxDimension);
            std::uniform_real_distribution<double> angleDist(-maxRotationDegree * 0.01745329, maxRotationDegree * 0.01745329);
            std::uniform_real_distribution<double> sizeDist(minScaleFactor, maxScaleFactor);

            // Initialize the outlier distribution
            std::uniform_real_distribution<double> outlierDist(-2*maxDimension, 2*maxDimension);

            // Do N repetitions
            for(int repetition=0; repetition<repetitions; repetition++) {
                std::cout << "Outlier Percentage: " << outlierPercentage << "; Model: " << model.getName() << "; Repetition: " << repetition << std::endl;

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
                int correspondenceCount = 1000;

                // Initialize an array of values and shuffle it
                std::vector<std::uint32_t> indices(model.size());
                iota(indices.begin(), indices.end(), 0); // Fill with values [0, 1, 2, ..]
                gaalign::shuffle_pcg_divisionless_with_slight_bias(indices.data(), indices.size());

                // Use the first N values to build correspondences
                std::vector<gaalign::Correspondence> correspondences(correspondenceCount);
                std::vector<gaalign::Correspondence> correspondingNormals(correspondenceCount);
                #pragma omp parallel for
                for(int i=0; i<correspondenceCount; i++) {
                    correspondences[i] = gaalign::Correspondence(transformed.getPoint(indices[i]), model.getPoint(indices[i]));
                    correspondingNormals[i] = gaalign::Correspondence(transformed.getNormal(indices[i]), model.getNormal(indices[i]));
                }

                // Set a specific percentage to outliers
                {
                    // Create a list of all correspondence indices and shuffle it
                    std::vector<std::size_t> corrIndices(correspondenceCount);
                    iota(corrIndices.begin(), corrIndices.end(), 0); // Fill with values [0, 1, 2, ..]
                    std::shuffle(corrIndices.begin(), corrIndices.end(), re);

                    // Loop over the first percentage of indices and make them outliers
                    for(int i=0; i<floor(correspondenceCount*outlierPercentage); i++) {
                        correspondences[corrIndices[i]].first = Eigen::Vector3d(outlierDist(re), outlierDist(re), outlierDist(re));
                        correspondences[corrIndices[i]].second = Eigen::Vector3d(outlierDist(re), outlierDist(re), outlierDist(re));
                        correspondingNormals[corrIndices[i]].first = Eigen::Vector3d(outlierDist(re), outlierDist(re), outlierDist(re)).normalized();
                        correspondingNormals[corrIndices[i]].second = Eigen::Vector3d(outlierDist(re), outlierDist(re), outlierDist(re)).normalized();
                    }
                }

                // For all algorithms
                for(int algorithmID=0; algorithmID<algorithms.size(); algorithmID++) {
                    // Run the algorithm
                    std::pair<Eigen::Matrix4d, double> result = algorithms[algorithmID]->calculateRegistration(correspondences, correspondingNormals);

                    // Apply the transformation to the point cloud
                    gaalign::PointCloud aligned = transformed;
                    #pragma omp parallel for
                    for(int i=0; i<transformed.size(); i++) {
                        // Apply transformation
                        Eigen::Vector4d point = result.first * Eigen::Vector4d(transformed.getPoint(i).x(), transformed.getPoint(i).y(), transformed.getPoint(i).z(), 1);
                        Eigen::Vector4d normal = result.first * Eigen::Vector4d(transformed.getNormal(i).x(), transformed.getNormal(i).y(), transformed.getNormal(i).z(), 1);

                        // Set transformed point / normal to point cloud
                        aligned.setPoint(i, Eigen::Vector3d(point.x(), point.y(), point.z()));
                        aligned.setNormal(i, Eigen::Vector3d(normal.x(), normal.y(), normal.z()));
                    }

                    // Measure the rmse between the aligned model and the original model
                    double squaredErrorSum = 0;
                    for(int i=0; i<aligned.size(); i++) {
                        squaredErrorSum += (aligned.getPoint(i) - model.getPoint(i)).squaredNorm();
                    }
                    double rmse = sqrt(squaredErrorSum / (double)transformed.size()) / maxDimension;

                    if(isnan(rmse)) {
                        std::cout << "Encountered a nan error for " << algorithms[algorithmID]->getName() << std::endl;
                        std::cout << result.first << std::endl;
                    }
                    else {
                        // Add the rmse and runtime to the output arrays
                        tempRuntimes[algorithmID].push_back(result.second);
                        tempErrors[algorithmID].push_back(rmse);
                    }


                }
            }
        }

        // Average the runtime and temp error per model for this noise level
        outlierPercentages.push_back(100*outlierPercentage);
        for(int i=0; i<algorithms.size(); i++) {
            double avgRuntime = avgVec(tempRuntimes[i]);
            double avgError = avgVec(tempErrors[i]);

            // Add to output
            rmses[i].push_back(avgError);
            runtimes[i].push_back(avgRuntime);
        }
    }

    // Save results to csv
    char buff[96];
    {
        time_t now = time(NULL);
        strftime(buff, 96, "%Y-%m-%d_%H-%M-%S_robustness_outlier", localtime(&now));

        // FIXME: This only works if the build directory is directly beneath the project root folder
        std::string csvPath = resourceFolder + "/../../results/" + buff + ".csv";

        std::cout << "Saving to '" << buff << "'" << std::endl;

        std::ofstream csvfile;
        csvfile.open(csvPath);

        // First write the noise values
        csvfile << "All" << ";" << "Outlier percentage" << ";";
        for (int i = 0; i < outlierPercentages.size(); i++) {
            if (i != 0) csvfile << ";";
            csvfile << outlierPercentages[i];
        }
        csvfile << std::endl;

        for (int algorithmID = 0; algorithmID < algorithms.size(); algorithmID++) {
            // RMSE
            csvfile << algorithms[algorithmID]->getName() << ";" << "RMSE" << ";";
            for (int i = 0; i < rmses[algorithmID].size(); i++) {
                if (i != 0) csvfile << ";";
                csvfile << rmses[algorithmID][i];
            }
            csvfile << std::endl;

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
    plt::figure_size(1200, 600);

    plt::subplot(1, 2, 1);
    plt::title("RMSE over outlier percentage");
    plt::xlabel("Outlier percentage [%]");
    plt::ylabel("RMSE");

    for(int algorithmID=0; algorithmID < algorithms.size(); algorithmID++) {
        plt::named_plot(algorithms[algorithmID]->getName(), outlierPercentages, rmses[algorithmID]);
    }

    // Add the legend
    plt::legend();

    // Plot the runtimes
    plt::subplot(1, 2, 2);
    plt::title("Runtime over outlier percentage");
    plt::xlabel("Outlier percentage [%]");
    plt::ylabel("Runtime [ms]");

    double maxRuntime = 0;
    for(int algorithmID=0; algorithmID < algorithms.size(); algorithmID++) {
        plt::named_plot(algorithms[algorithmID]->getName(), outlierPercentages, runtimes[algorithmID]);

        for(const auto& runtime : runtimes[algorithmID]) {
            if(runtime > maxRuntime) {
                maxRuntime = runtime;
            }
        }
    }

    // Set the limits
    plt::ylim<double>(0, maxRuntime);

    // Add the legend
    plt::legend();

    // Save to file
    plt::save(resourceFolder + "/../../results/" + buff + ".png");

    // Show the plot
    if(visualize) {
        plt::show();
    }
}
