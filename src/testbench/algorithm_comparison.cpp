//
// Created by Kai on 11.04.2022.
//

#include "algorithm_comparison.h"

#include <vector>
#include <iostream>
#include <random>
#include <fstream>

#include "boost/filesystem.hpp"

#include "geometry/point_cloud.h"
#include "synthetic_data_generator.h"

#ifdef ENABLE_VISUALIZATION
#include "visualization/visualize.h"
#endif

// Algorithms
#include "testbench/wrappers/algorithm_wrapper.h"
#include "testbench/wrappers/gaalign_wrapper.h"
#include "testbench/wrappers/pcl_icp_wrapper.h"
#include "testbench/wrappers/opencv_icp_wrapper.h"
#include "testbench/wrappers/fgr_wrapper.h"
#include <testbench/wrappers/super4pcs_wrapper.h>
#include "testbench/wrappers/ga_lms_wrapper.h"

// Matplotlib
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

namespace {
    std::string zfill(const int &value, const int &zeros) {
        // Convert the value to a string
        std::string str = std::to_string(value);

        if(zeros > str.size())
            str.insert(0, zeros - str.size(), '0');

        return str;
    }
}

void compare_algorithms(double noiseLevel, std::string resourceFolder, bool visualize, bool saveDatasets) {
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

    // Find the path to the third party executables
    std::string thirdPartyFolder = boost::filesystem::absolute(boost::filesystem::path(resourceFolder)).string() + "/../3rdparty";
    if(!boost::filesystem::exists(thirdPartyFolder)) {
        std::cout << "Error: The 3rdparty folder containing the external applications was not found!" << std::endl;
        return;
    }

    // Create the datasets folder (if we want to save it out)
    std::string datasetsRootFolder = boost::filesystem::absolute(boost::filesystem::path(resourceFolder)).string() + "/../../Datasets/";
    std::string datasetFolder = datasetsRootFolder + "noise_" + std::to_string(noiseLevel) + "/";
    std::string datasetSourceFolder = datasetsRootFolder + "noise_" + std::to_string(noiseLevel) + "/source";
    std::string datasetSourceGTFolder = datasetsRootFolder + "noise_" + std::to_string(noiseLevel) + "/source_gt";
    std::string datasetTargetFolder =  datasetsRootFolder + "noise_" + std::to_string(noiseLevel) + "/target";

    if(saveDatasets) {
        if(!boost::filesystem::exists(datasetsRootFolder)) {
            boost::filesystem::create_directory(datasetsRootFolder);
        }

        // Remove the previous dataset
        if(boost::filesystem::exists(datasetFolder)) {
            boost::filesystem::remove_all(datasetFolder);
        }

        // Create the new folder
        boost::filesystem::create_directory(datasetFolder);

        // Also create the source and target folders
        boost::filesystem::create_directory(datasetSourceFolder);
        boost::filesystem::create_directory(datasetTargetFolder);
        boost::filesystem::create_directory(datasetSourceGTFolder);
    }

    // Initialize the algorithms that should be compared
    std::vector<std::shared_ptr<gaalign::RegistrationAlgorithmWrapper>> algorithms;
    algorithms.push_back(std::make_shared<gaalign::GaalignWrapper>());
    algorithms.push_back(std::make_shared<gaalign::PCLICPWrapper>(gaalign::PCL_ICP_VARIANT::SVD_BASED));
    algorithms.push_back(std::make_shared<gaalign::PCLICPWrapper>(gaalign::PCL_ICP_VARIANT::POINT_TO_PLANE));
    algorithms.push_back(std::make_shared<gaalign::PCLICPWrapper>(gaalign::PCL_ICP_VARIANT::SYMMETRIC_POINT_TO_PLANE));
    algorithms.push_back(std::make_shared<gaalign::OpenCVWrapper>());
    algorithms.push_back(std::make_shared<gaalign::FGRWrapper>(thirdPartyFolder));
    algorithms.push_back(std::make_shared<gaalign::GA_LMS_WRAPPER>(false));
    algorithms.push_back(std::make_shared<gaalign::GA_LMS_WRAPPER>(true));
    //algorithms.push_back(std::make_shared<gaalign::Super4PCSWrapper>(thirdPartyFolder));

    // Init the result arrays
    std::vector<std::vector<double>> rmsePerAlgorithm;
    rmsePerAlgorithm.reserve(algorithms.size());
    for(int i=0; i<algorithms.size(); i++) {
        rmsePerAlgorithm.emplace_back();
    }

    // Settings
    int repetitionsCount = 100; // Increase to 50 or more for smoother results -> We used 100 for the paper
    bool useRandomizedSubsampling = true;

    // Initialize the random distributions
    srand((std::random_device())());
    std::mt19937 re((std::random_device())());
    std::uniform_real_distribution<double> overlapDistribution(0.2, 1);
    std::uniform_real_distribution<double> subsampleDistribution(0.1, 1);

    // Settings for the random transform
    double maxRelativeTranslation = 0.05;
    double maxRotationDegree = 5;
    double minScaleFactor = 1;
    double maxScaleFactor = 1;

    // Loop over all models
    int counter = 0;
    for(const auto& model : models) {
        // Get the maximum dimension of the original point cloud, to calculate the noise scale
        double maxDimension = model.getMaxDimension();

        // Initialize model specific distribution
        std::uniform_real_distribution<double> translationDist(-maxRelativeTranslation*maxDimension, maxRelativeTranslation*maxDimension);
        std::uniform_real_distribution<double> angleDist(-maxRotationDegree*0.01745329, maxRotationDegree*0.01745329);
        std::uniform_real_distribution<double> sizeDist(minScaleFactor, maxScaleFactor);

        // Do N repetitions
        for(int repetition=0; repetition<repetitionsCount; repetition++) {
            std::cout << "\n[" << model.getName() << "] repetition " << repetition + 1 << "/" << repetitionsCount << std::endl;
            counter++;

            // Choose a random overlap for slicing
            double overlap = overlapDistribution(re);
            std::cout << "Using " << 100*overlap << "% overlap for slicing" << std::endl;

            std::pair<gaalign::PointCloud, gaalign::PointCloud> sliced;
            if(useRandomizedSubsampling) {
                // Subsample to a random number of points
                double subsamplePercentage = subsampleDistribution(re);
                std::cout << "Subsampling to " << 100*subsamplePercentage << "% of points" << std::endl;
                gaalign::PointCloud subsampled = model.subsample((int)(subsamplePercentage*model.size()));

                // Slice the models with a random overlap
                sliced = slicePointCloud(subsampled, overlap);
            }
            else {
                // Slice the models with a random overlap
                sliced = slicePointCloud(model, overlap);
            }

            // Add gaussian noise to both point clouds
            gaalign::PointCloud source_disturbed = disturbWithGaussianNoise(sliced.first, noiseLevel*maxDimension);
            gaalign::PointCloud target_disturbed = disturbWithGaussianNoise(sliced.second, noiseLevel*maxDimension);

            // Disturb with outliers
            //source_disturbed = disturbWithOutliers(source_disturbed, 0.1, maxDimension);
            //target_disturbed = disturbWithOutliers(target_disturbed, 0.1, maxDimension);

            // Randomly transform the source point cloud
            gaalign::PointCloud source_transformed = source_disturbed;
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
                for(int i=0; i<source_transformed.size(); i++) {
                    // TODO: Fix scaling issues of GAAlign before reenabling this
                    source_transformed.setPoint(i, scaling * rotationMatrix * source_disturbed.getPoint(i) + translation);
                    source_transformed.setNormal(i, rotationMatrix * source_disturbed.getNormal(i));
                }
            }

            // Reset both bounding boxes
            source_transformed.resetBoundingBox();
            target_disturbed.resetBoundingBox();

            // Save out both point clouds
            if(saveDatasets) {
                source_disturbed.saveToPly(datasetSourceGTFolder + "/" + zfill(counter, 5) + ".ply");
                source_transformed.saveToPly(datasetSourceFolder + "/" + zfill(counter, 5) + ".ply");
                target_disturbed.saveToPly(datasetTargetFolder + "/" + zfill(counter, 5)+ ".ply");
            }

            // Debug vis
            //visualizeCorrespondences3D(source_transformed, target_disturbed, {});

            // For each of the algorithms
            for(int algorithmID=0; algorithmID < algorithms.size(); algorithmID++) {
                std::cout << "\nExecuting " << algorithms[algorithmID]->getName() << std::endl;

                // Calculate the alignment from source_transformed to target
                gaalign::PointCloud result = algorithms[algorithmID]->calculateRegistration(source_transformed, target_disturbed, overlap);

                //visualizeCorrespondences3D(result, target_disturbed, {});

                // Compare source_transformed with source_disturbed -> Calculate RMSE
                double squaredErrorSum = 0;
                for(int i=0; i<result.size(); i++) {
                    squaredErrorSum += (result.getPoint(i) - source_disturbed.getPoint(i)).squaredNorm();
                }

                double rmse = sqrt(squaredErrorSum / (double)result.size()) / maxDimension;

                // Add resulting metrics to output
                rmsePerAlgorithm[algorithmID].push_back(rmse);
                std::cout << "====> RMSE: " << rmse << std::endl;
            }
        }
    }

    // Print the rmses
    for(int algorithmID=0; algorithmID < algorithms.size(); algorithmID++) {
        std::cout << algorithms[algorithmID]->getName() << ": [";
        for(int i=0; i<rmsePerAlgorithm[algorithmID].size(); i++) {
            if(i!=0) std::cout << ", ";
            std::cout << rmsePerAlgorithm[algorithmID][i];
        }
        std::cout << "]" << std::endl;
    }

    // Also save them as csv
    char buff[96];
    time_t now = time(NULL);
    strftime(buff, 96, "%Y-%m-%d_%H-%M-%S_testbench_syncthetic", localtime(&now));

    // FIXME: This only works if the build directory is directly beneath the project root folder
    std::string csvPath = resourceFolder + "/../../results/" + buff + ".csv";

    std::cout << "Saving to '" << buff << "'" << std::endl;

    std::ofstream csvfile;
    csvfile.open (csvPath);
    for(int algorithmID=0; algorithmID < algorithms.size(); algorithmID++) {
        csvfile << algorithms[algorithmID]->getName() << ";";
        for(int i=0; i<rmsePerAlgorithm[algorithmID].size(); i++) {
            if(i!=0) csvfile << ";";
            csvfile << rmsePerAlgorithm[algorithmID][i];
        }
        csvfile << std::endl;
    }
    csvfile.close();


    // Calculate and plot alpha-recall
    // Create a figure
    plt::figure_size(1200, 1000);
    plt::title("α-Recall over RMSE for noise strength σ=" + std::to_string(noiseLevel));
    plt::xlabel("RMSE");
    plt::ylabel("α-Recall");

    for(int algorithmID=0; algorithmID < algorithms.size(); algorithmID++) {
        std::vector<double> x;
        std::vector<double> y;

        // Loop over a range of noise levels
        for(double alpha=0; alpha <= 0.06; alpha+=0.0001) {
            // Add the x value
            x.push_back(alpha);

            // Loop over all rmse values of this algorithm and count how many are smaller than alpha
            int smallerThanAlpha = 0;
            for(int i=0; i<rmsePerAlgorithm[algorithmID].size(); i++) {
                if(rmsePerAlgorithm[algorithmID][i] < alpha) {
                    smallerThanAlpha++;
                }
            }

            double alphaRecall = ((double) smallerThanAlpha)/((double) rmsePerAlgorithm[algorithmID].size());
            y.push_back(alphaRecall);
        }

        plt::named_plot(algorithms[algorithmID]->getName(), x, y);
    }

    // Add the legend
    plt::legend();

    // Save the plot
    plt::save(resourceFolder + "/../../results/" + buff + ".png");

    // Show the plot
    if(visualize) {
        plt::show();
    }

}
