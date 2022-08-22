//
// Created by Kai on 11.04.2022.
//

#include "synthetic_data_generator.h"
#include <random>

std::pair<gaalign::PointCloud, gaalign::PointCloud> slicePointCloud(const gaalign::PointCloud &pointCloud, double overlap) {

    // 1.) Choose a random normal for the plane in which we want to cut
    Eigen::Vector3d normal = Eigen::Vector3d::Random();
    normal.normalize();

    // 2.) Project all points onto the direction of the normal
    double xMin = 1e16;
    double xMax = -1e16;
    for(int i=0; i<pointCloud.size(); i++) {
        double proj = normal.dot(pointCloud.getPoint(i));
        if(proj < xMin) {
            xMin = proj;
        }
        if(proj > xMax) {
            xMax = proj;
        }
    }

    // 3.) Calculate the two cut thresholds
    std::mt19937 re((std::random_device())());
    std::uniform_real_distribution<double> dist(xMin+0.05*(xMax-xMin), xMax-0.05*(xMax-xMin)-overlap*(xMax-xMin));
    double thresh1 = dist(re);
    double thresh2 = overlap*(xMax-xMin) + thresh1;

    //std::cout << "[" << xMin << ", " << thresh1 << ", " << thresh2 << ", " << xMax << "] -> Overlap: " << overlap << std::endl;

    // Sanity check
    if(thresh2 < thresh1) {
        std::cout << "Slicing produced invalid thresholds!" << std::endl;
        throw std::exception("Slicing produced invalid thresholds!");
    }

    // Create the first half by using the first threshold to cut
    gaalign::PointCloud firstHalf;
    gaalign::PointCloud secondHalf;
    for(int i=0; i<pointCloud.size(); i++) {
        double proj = normal.dot(pointCloud.getPoint(i));

        // Part of the first half
        if(proj < thresh2) {
            firstHalf.addIndex(firstHalf.size());
            firstHalf.addPoint(pointCloud.getPoint(i));
            firstHalf.addNormal(pointCloud.getNormal(i));
        }

        // part of the second half
        if(proj > thresh1) {
            secondHalf.addIndex(secondHalf.size());
            secondHalf.addPoint(pointCloud.getPoint(i));
            secondHalf.addNormal(pointCloud.getNormal(i));
        }

    }

    // Update the bounding boxes
    firstHalf.resetBoundingBox();
    secondHalf.resetBoundingBox();

    return std::make_pair(firstHalf, secondHalf);
}

gaalign::PointCloud disturbWithGaussianNoise(const gaalign::PointCloud &pointCloud, double sigma) {
    // Initialize the distribution and random engine
    std::normal_distribution<double> distribution(0, sigma);
    std::default_random_engine re;

    // Initialize the output point cloud
    gaalign::PointCloud disturbed = pointCloud;

    #pragma omp parallel for
    for(int i=0; i<pointCloud.size(); i++) {
        disturbed.setPoint(i, pointCloud.getPoint(i) + Eigen::Vector3d(distribution(re), distribution(re), distribution(re)));
        disturbed.setNormal(i, (pointCloud.getNormal(i) + Eigen::Vector3d(distribution(re), distribution(re), distribution(re))).normalized());
    }

    return disturbed;
}

gaalign::PointCloud disturbWithOutliers(const gaalign::PointCloud &pointCloud, double percentage, double maxDimension) {
    // Initialize the and random engine
    std::default_random_engine re;

    // Intit the outlier distribution
    std::uniform_real_distribution<double> outlierDist(-0.4f*maxDimension, 0.4f*maxDimension);

    // Initialize the output point cloud
    gaalign::PointCloud disturbed = pointCloud;

    // Set a specific percentage to outliers
    {
        // Create a list of all correspondence indices and shuffle it
        std::vector<std::size_t> indices(pointCloud.size());
        iota(indices.begin(), indices.end(), 0); // Fill with values [0, 1, 2, ..]
        std::shuffle(indices.begin(), indices.end(), re);

        int maxInd = floor(pointCloud.size()*percentage);

        // Loop over the first percentage of indices and make them outliers
        #pragma omp parallel for
        for(int i=0; i<maxInd; i++) {
            disturbed.setPoint(indices[i], disturbed.getPoint(indices[i]) + Eigen::Vector3d(outlierDist(re), outlierDist(re), outlierDist(re)));
        }
    }

    return disturbed;
}

