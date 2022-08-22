//
// Created by Kai on 15.04.2022.
//

#include "gradient_descent_hyperparameter_optimization.h"

#include <iostream>
#include <vector>
#include <random>
#include <fstream>
#include <chrono>

#include <geometry/point_cloud.h>
#include <optimization/gradient_descent/gradient_descent.h>
#include <testbench/common.h>

// Matplotlib
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

void gaalign::findOptimalHyperParameters(const std::string& resourceFolder) {
    std::cout << "Starting hyperparameter optimization!" << std::endl;

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
        models[i] = models[i].subsample(10000);
    }

    // Genetic search parameters
    int populationSize = 100; // How many individuals we want to have during testing
    int parentsCount = 10; // How many of the best indiviuals are kept out of the whole population
    int maxIterations = 50; // How many iterations we should do as a maximum

    // Distributions for the initial parameters
    std::mt19937 re((std::random_device())());
    std::uniform_int_distribution<int> dist_iterations(10, 1000);
    std::uniform_int_distribution<int> dist_trianglesPerIteration(10, 1000);
    std::uniform_real_distribution<double> dist_stepSize(0.01, 0.5);
    std::uniform_real_distribution<double> dist_edgeLength(0.01, 0.4);
    std::uniform_real_distribution<double> dist_angle(1, 30);

    // Distribution for recombination
    std::uniform_int_distribution<int> dist_parentSelection(0, parentsCount-1);
    std::uniform_real_distribution<double> dist_combinationFactor(0.2, 0.8);

    // Mutation distribution
    std::normal_distribution<double> normalDistribution(0, 1);

    // Initialize vectors to store the best and worst fitness values of each generations
    std::vector<double> bestFitnessValues;
    std::vector<GaalignHyperParameters> bestHyperparameters;
    std::vector<double> worstFitnessValues;
    std::vector<double> avgFitnessValues;
    std::vector<double> changeValues;

    // Randomly initialize N sets of parents
    std::vector<GaalignHyperParameters> parents(parentsCount);
    for(int i=0; i<parentsCount; i++) {
        parents[i].maxIterations = dist_iterations(re);
        parents[i].trianglesPerIteration = dist_trianglesPerIteration(re);
        parents[i].stepSize = dist_stepSize(re);
        parents[i].minTriangleRelativeEdgeLength = dist_edgeLength(re);
        parents[i].minTriangleAngle = dist_angle(re);
        //parents[i].print();
    }
    std::cout << "Initialized starting population!" << std::endl;

    for(int generation=0; generation<maxIterations; generation++) {
        std::cout << std::endl << "=======================" << std::endl;
        std::cout << "=== GENERATION " << generation << " ===";
        if(generation < 10) std::cout << "=";
        if(generation < 100) std::cout << "=";
        if(generation < 1000) std::cout << "=";
        std::cout << std::endl;
        std::cout << "=======================" << std::endl;

        // Create a new population based on the parents
        std::vector<GaalignHyperParameters> population;
        for(int i=0; i<populationSize; i++) {
            const GaalignHyperParameters& parent1 = parents[dist_parentSelection(re)];
            const GaalignHyperParameters& parent2 = parents[dist_parentSelection(re)];
            double fac = dist_combinationFactor(re);

            // Create the new children
            GaalignHyperParameters child;
            child.maxIterations = (int)round(fac*parent1.maxIterations + (1-fac)*parent2.maxIterations);
            child.trianglesPerIteration = (int)round(fac*parent1.trianglesPerIteration + (1-fac)*parent2.trianglesPerIteration);
            child.stepSize = fac*parent1.stepSize + (1-fac)*parent2.stepSize;
            child.minTriangleRelativeEdgeLength = fac*parent1.minTriangleRelativeEdgeLength + (1-fac)*parent2.minTriangleRelativeEdgeLength;
            child.minTriangleAngle = fac*parent1.minTriangleAngle + (1-fac)*parent2.minTriangleAngle;

            // We also need to lerp the mutation step size
            child.sigma_maxIterations = (int)round(fac*parent1.sigma_maxIterations + (1-fac)*parent2.sigma_maxIterations);
            child.sigma_triangles = (int)round(fac*parent1.sigma_triangles + (1-fac)*parent2.sigma_triangles);
            child.sigma_stepSize = fac*parent1.sigma_stepSize + (1-fac)*parent2.sigma_stepSize;
            child.sigma_minTriangleRelativeEdgeLength = fac*parent1.sigma_minTriangleRelativeEdgeLength + (1-fac)*parent2.sigma_minTriangleRelativeEdgeLength;
            child.sigma_minTriangleAngle = fac*parent1.sigma_minTriangleAngle + (1-fac)*parent2.sigma_minTriangleAngle;


            population.push_back(child);
        }

        // Choose a global mutation factor for all mutations in this iteration
        double mutationFac = exp(normalDistribution(re));

        // Mutate the population
        #pragma omp parallel for
        for(int i=0; i<populationSize; i++) {
            // First mutate the mutation step sizes
            population[i].sigma_maxIterations = std::clamp(population[i].sigma_maxIterations*exp(normalDistribution(re))*mutationFac, 0.1, 100.0);
            population[i].sigma_triangles = std::clamp(population[i].sigma_triangles*exp(normalDistribution(re))*mutationFac, 0.1, 100.0);
            population[i].sigma_stepSize = std::clamp(population[i].sigma_stepSize*exp(normalDistribution(re))*mutationFac, 0.001, 0.1);
            population[i].sigma_minTriangleRelativeEdgeLength = std::clamp(population[i].sigma_minTriangleRelativeEdgeLength*exp(normalDistribution(re))*mutationFac, 0.001, 0.1);
            population[i].sigma_minTriangleAngle = std::clamp(population[i].sigma_minTriangleAngle*exp(normalDistribution(re))*mutationFac, 0.01, 10.0);

            // Then mutate the actual values
            population[i].maxIterations = (int)round(std::clamp(population[i].maxIterations + normalDistribution(re)*population[i].sigma_maxIterations, 1.0, 1000.0));
            population[i].trianglesPerIteration = (int)round(std::clamp(population[i].trianglesPerIteration + normalDistribution(re)*population[i].sigma_triangles, 1.0, 1000.0));
            population[i].stepSize = std::clamp(population[i].stepSize + normalDistribution(re)*population[i].sigma_stepSize, 0.0001, 1.0);
            population[i].minTriangleRelativeEdgeLength = std::clamp(population[i].minTriangleRelativeEdgeLength + normalDistribution(re)*population[i].sigma_minTriangleRelativeEdgeLength, 0.001, 1.0);
            population[i].minTriangleAngle = std::clamp(population[i].minTriangleAngle + normalDistribution(re)*population[i].sigma_minTriangleAngle, 1.0, 45.0);
        }

        // Evaluate the fitness of all individuals (smaller is better)
        std::vector<double> fitness(populationSize);
        for(int i=0; i<populationSize; i++) {
            // Evaluate robustness against noise
            std::pair<double, double> noiseResults = evaluateRobustnessAgainstNoise(population[i], models, false);
            std::pair<double, double> outlierResults = evaluateRobustnessAgainstOutliers(population[i], models, false);

            // Extract the values
            double errorNoise = noiseResults.first;
            double timeNoise = noiseResults.second;
            double errorOutlier = outlierResults.first;
            double timeOutlier = outlierResults.second;

            // Create a shared fitness value
            fitness[i] = 25000*errorNoise + errorOutlier + timeNoise + timeOutlier;
        }

        // Keep only the best N individuals as parents for the next generation
        // initialize original index locations
        std::vector<std::size_t> indices(populationSize);
        iota(indices.begin(), indices.end(), 0); // Fill with values [0, 1, 2, ..]

        // Sort the indices based on the fitness values
        stable_sort(indices.begin(), indices.end(), [&fitness](size_t i1, size_t i2) {return fitness[i1] < fitness[i2];});

        // Set the new parents based on the first values
        for(int i=0; i<parentsCount; i++) {
            parents[i] = population[indices[i]];
        }

        // Print the current status
        std::cout << "==> Current best hyperparameters with fitness " << fitness[indices[0]] << ": ";
        parents[0].print();
        bestHyperparameters.push_back(parents[0]);

        // Buffer the best and worst fitness values of all generations
        bestFitnessValues.push_back(fitness[indices[0]]);
        worstFitnessValues.push_back(fitness[indices[populationSize-1]]);
        avgFitnessValues.push_back(avg(fitness));

        std::cout << "==> Best: " << bestFitnessValues[generation] << " < Avg: " << avgFitnessValues[generation] << " < Worst: " << worstFitnessValues[generation] << std::endl;

        // Termination criterion: Check the best and worst fitness values of the last 10 generations
        if(generation > 10) {
            double worst = -1e16;
            double best = 1e16;

            for(int i=0; i<10; i++) {
                if(worstFitnessValues[generation - i] > worst) worst = worstFitnessValues[generation - i];
                if(bestFitnessValues[generation - i] < best) best = bestFitnessValues[generation - i];
            }

            // Check the distance of the worst and best fitness as termination criterion
            double change = abs((worst - best)/best);
            changeValues.push_back(change);
            std::cout << "==> Relative change (epsilon) over last 10 generations: " << change << std::endl;
            if(change < 1e-3) {
                std::cout << "========> The algorithm converged after " << generation << " generations!" << std::endl;
                break;
            }
        }
        else {
            // Zeros
            changeValues.push_back(0);
        }

        if(generation > 100) {
            break;
        }

    }

    // Get a timestamp
    char buff[96];
    time_t now = time(NULL);
    strftime(buff, 96, "%Y-%m-%d_%H-%M-%S_hyperparameter optimization.csv", localtime(&now));

    // FIXME: This only works if the build directory is directly beneath the project root folder
    std::string csvPath = resourceFolder + "/../../results/" + buff;

    std::cout << "Saving to '" << buff << "'" << std::endl;

    std::ofstream csvfile;
    csvfile.open (csvPath);
    csvfile << "Generation;Best;Worst;Avg;Epsilon;maxIterations;trianglesPerIteration;stepSize;minTriangleRelativeEdgeLength;minTriangleAngle" << std::endl;
    for(int i=0; i<bestFitnessValues.size(); i++) {
        csvfile << i << ";" << bestFitnessValues[i] << ";" << worstFitnessValues[i] << ";" << avgFitnessValues[i] << ";" << changeValues[i] << ";";
        csvfile << bestHyperparameters[i].maxIterations << ";";
        csvfile << bestHyperparameters[i].trianglesPerIteration << ";";
        csvfile << bestHyperparameters[i].stepSize << ";";
        csvfile << bestHyperparameters[i].minTriangleRelativeEdgeLength << ";";
        csvfile << bestHyperparameters[i].minTriangleAngle << std::endl;
    }
    csvfile.close();

    // Plot the error
    {
        // Create a figure
        plt::figure_size(1200, 1000);
        plt::title("Hyperparameter Error over the generations");
        plt::xlabel("Generation");
        plt::ylabel("Error");

        std::vector<double> x;
        for(int i=0; i<bestFitnessValues.size(); i++) {
            x.push_back(i);
        }

        plt::named_plot("Best Hyperparameters", x, bestFitnessValues);
        plt::named_plot("Worst Hyperparameters", x, worstFitnessValues);
        plt::named_plot("Average Hyperparameters", x, avgFitnessValues);

        // Add the legend
        plt::legend();
    }

    // Plot the change
    {
        // Create a figure
        plt::figure_size(1200, 1000);
        plt::title("Relative change $\\epsilon$ over generations");
        plt::xlabel("Generation");
        plt::ylabel("$\\epsilon$");

        std::vector<double> x;
        for(int i=0; i<changeValues.size(); i++) {
            x.push_back(i);
        }

        plt::named_plot("Best Hyperparameters", x, changeValues);

        // Add the legend
        plt::legend();
    }

    // Show the plot
    plt::show();






}

std::pair<double, double> gaalign::evaluateRobustnessAgainstNoise(const gaalign::GaalignHyperParameters &params, const std::vector<gaalign::PointCloud> &models, bool visualize) {
    // Initialize an optimizer using the hyperparameters
    gaalign::GradientDescentOptimizer optimizer;
    optimizer.getSettings().maxIterations = params.maxIterations;
    optimizer.getSettings().trianglesPerIteration = params.trianglesPerIteration;
    optimizer.getSettings().stepSize = params.stepSize;
    optimizer.getSettings().minTriangleRelativeEdgeLength = params.minTriangleRelativeEdgeLength;
    optimizer.getSettings().minTriangleAngle = params.minTriangleAngle;

    // Set the number of repetitions
    int repetitions = 3;
    double minNoise = 0;
    double maxNoise = 0.31;
    double stepSize = 0.1;

    // Set the different transformations that are tested
    std::vector<double> translations = {0, 0.1, 0.5, 1};
    std::vector<double> angles = {0, 2, 5, 10, 45, 90};

    std::vector<std::pair<double, double>> transformPairs;
    for(const auto& t : translations) {
        for(const auto& a : angles) {
            transformPairs.emplace_back(t, a);
        }
    }

    // Collect the results
    std::vector<double> noiseValues;
    std::vector<double> resultingRMSE;
    std::vector<double> runtimes;

    // Initialize the random engine
    std::mt19937 re((std::random_device())());

    // Initialize the distribution of the number of correspondences
    std::uniform_int_distribution<int> correspondenceCountDist(100, 5000);

    // Go from a small amount of noise to a large amount of noise
    for(double noiseLevel=minNoise; noiseLevel<=maxNoise; noiseLevel+=stepSize) {
        //std::cout << "Noise level " << noiseLevel << std::endl;

        // Collect all RMSE values of this noise level
        std::vector<double> tempRMSE;

        // Loop over all models
        #pragma omp parallel for schedule(dynamic)
        for(int modelID=0; modelID<models.size(); modelID++) {
            const auto& model = models[modelID];

            // Get the maximum dimension of the original point cloud, to calculate the noise scale
            double maxDimension = model.getMaxDimension();

            // Initialize the random distribution for this noise level and model
            std::normal_distribution<double> noiseDistribution(0, noiseLevel*maxDimension);

            for(const auto& transformation : transformPairs) {

                // Do N repetitions
                //#pragma omp parallel for schedule(dynamic)
                for(int iter=0; iter<repetitions; iter++) {
                    /*#pragma omp critical
                    {
                        std::cout << "Model " << model.getName() << " repetition " << iter << std::endl;
                    }*/
                    //std::chrono::steady_clock::time_point beginRep = std::chrono::steady_clock::now();

                    // Apply a random transformation to the model
                    gaalign::PointCloud transformed = model;
                    {
                        Eigen::AngleAxisd rollAngle(transformation.second*0.01745329, Eigen::Vector3d::UnitZ());
                        Eigen::AngleAxisd yawAngle(-transformation.second*0.01745329, Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd pitchAngle(transformation.second*0.01745329, Eigen::Vector3d::UnitX());
                        Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
                        Eigen::Matrix3d rotationMatrix = q.matrix();
                        double scaling = 1.0; // Currently not part of the tests

                        // Random translation
                        Eigen::Vector3d translation(transformation.first, -transformation.first, transformation.first);

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
                    #pragma omp parallel for
                    for(int i=0; i<correspondenceCount; i++) {
                        correspondences[i] = gaalign::Correspondence(transformed.getPoint(indices[i]), model.getPoint(indices[i]));
                    }

                    // Add noise to the correspondences (not the model)
                    #pragma omp parallel for
                    for(int i=0; i<correspondenceCount; i++) {
                        correspondences[i].first += Eigen::Vector3d(noiseDistribution(re), noiseDistribution(re), noiseDistribution(re));
                        correspondences[i].second += Eigen::Vector3d(noiseDistribution(re), noiseDistribution(re), noiseDistribution(re));
                    }

                    // Start the time measurement
                    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

                    // Estimate a motor from the correspondences
                    gaalign::Motor motor = optimizer.optimize(correspondences);

                    auto end = std::chrono::high_resolution_clock::now();
                    double timeMS = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/1000000.0;
                    #pragma omp critical
                    {
                        runtimes.push_back(timeMS);
                    };


                    // Apply that motor to the transformed model
                    #pragma omp parallel for
                    for(int i=0; i<transformed.size(); i++) {
                        transformed.setPoint(i, transformPointWithMotor(transformed.getPoint(i), motor));
                        transformed.setNormal(i, transformPointWithMotor(transformed.getNormal(i), motor));
                    }

                    // Measure the rmse between the aligned model and the original model
                    double squaredErrorSum = 0;
                    for(int i=0; i<transformed.size(); i++) {
                        squaredErrorSum += (transformed.getPoint(i) - model.getPoint(i)).squaredNorm();
                    }
                    double rmse = sqrt(squaredErrorSum / (double)transformed.size()) / maxDimension;

                    #pragma omp critical
                    {
                        tempRMSE.push_back(rmse);
                    };


                    /*auto endRep = std::chrono::high_resolution_clock::now();
                    double timeMSRep = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(endRep - beginRep).count())/1000000.0;
                    std::cout << "Repetition took " << timeMSRep << " ms (Optimization part took " << timeMS << " ms)" << std::endl;*/
                }
            }
        }

        // Average the RMSEs
        double avgRMSE = avg(tempRMSE);

        // Add to output
        noiseValues.push_back(noiseLevel);
        resultingRMSE.push_back(avgRMSE);
    }

    // Average the runtime
    double avgRuntime = avg(runtimes);

    // Plot the rmses
    if(visualize) {
        // Create a figure
        plt::figure_size(1200, 1000);
        plt::title("Average RMSE for noisy correspondences");
        plt::xlabel("Noise Level");
        plt::ylabel("RMSE");
        plt::named_plot("RMSE", noiseValues, resultingRMSE, "");

        plt::show();
    }

    // Calculate the area under the curve as a single metric
    double auc = 0;
    for(int i=0; i<resultingRMSE.size()-1; i++) {
        auc += 0.5*(noiseValues[i+1]-noiseValues[i])*(resultingRMSE[i+1]+resultingRMSE[i]);
    }

    std::cout << "[" << auc << "; " << avgRuntime << "]" << std::endl;

    return {auc, avgRuntime};
}

std::pair<double, double> gaalign::evaluateRobustnessAgainstOutliers(const gaalign::GaalignHyperParameters &params, const std::vector<gaalign::PointCloud> &models, bool visualize) {
    return {};
}

void gaalign::evaluateRobustnessOfSpecificHyperparameters(const gaalign::GaalignHyperParameters &params, const std::string& resourceFolder) {
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

    // Noise
    gaalign::evaluateRobustnessAgainstNoise(params, models, true);

    // Outliers
}
