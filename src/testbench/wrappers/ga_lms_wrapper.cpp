//
// Created by Kai on 03.05.2022.
//

#include "ga_lms_wrapper.h"
#include "testbench/wrappers_optim/ga_lms_optimization_wrapper.h"
#include "correspondence/feature/feature_search.h"
#include "correspondence/distance/distance_search.h"

gaalign::PointCloud gaalign::GA_LMS_WRAPPER::calculateRegistration(const gaalign::PointCloud &source, const gaalign::PointCloud &target,
                                               double overlap) const {

    // Initialize an optimizer
    GA_LMS_OptimizationWrapper wrapper(m_useSteepestDescent);

    // Do a feature search
    std::shared_ptr<gaalign::FeatureSearch> featureSearch = std::make_shared<gaalign::FeatureSearch>();
    featureSearch->getSettings().visualizeKeypoints = false;
    featureSearch->getSettings().visualizeMatches = false;

    // Calculate correspondences using feature search
    std::vector<gaalign::Correspondence> correspondences = featureSearch->compute(source, target);
    std::pair<Eigen::Matrix4d, double> transformCoarse = wrapper.calculateRegistration(correspondences, {});

    // Apply the matrix to all points in the point cloud
    gaalign::PointCloud resultCoarse = source;
    #pragma omp parallel for
    for(int i=0; i<source.size(); i++) {
        // Apply transformation
        Eigen::Vector4d point = transformCoarse.first * Eigen::Vector4d(source.getPoint(i).x(), source.getPoint(i).y(), source.getPoint(i).z(), 1);
        Eigen::Vector4d normal = transformCoarse.first * Eigen::Vector4d(source.getNormal(i).x(), source.getNormal(i).y(), source.getNormal(i).z(), 1);

        // Set transformed point / normal to point cloud
        resultCoarse.setPoint(i, Eigen::Vector3d(point.x(), point.y(), point.z()));
        resultCoarse.setNormal(i, Eigen::Vector3d(normal.x(), normal.y(), normal.z()));
    }

    // Create a distance search
    std::shared_ptr<gaalign::DistanceSearch> distanceSearch = std::make_shared<gaalign::DistanceSearch>();
    distanceSearch->init(resultCoarse, target);

    // Calculate the correspondences
    std::vector<gaalign::Correspondence> correspondencesFine = distanceSearch->compute(resultCoarse, target);

    // Calculate the registration
    std::pair<Eigen::Matrix4d, double> transformFine = wrapper.calculateRegistration(correspondencesFine, {});

    // Apply the matrix to all points in the point cloud
    gaalign::PointCloud resultFine = source;
    #pragma omp parallel for
    for(int i=0; i<source.size(); i++) {
        // Apply transformation
        Eigen::Vector4d point = transformFine.first * Eigen::Vector4d(resultCoarse.getPoint(i).x(), resultCoarse.getPoint(i).y(), resultCoarse.getPoint(i).z(), 1);
        Eigen::Vector4d normal = transformFine.first * Eigen::Vector4d(resultCoarse.getNormal(i).x(), resultCoarse.getNormal(i).y(), resultCoarse.getNormal(i).z(), 1);

        // Set transformed point / normal to point cloud
        resultFine.setPoint(i, Eigen::Vector3d(point.x(), point.y(), point.z()));
        resultFine.setNormal(i, Eigen::Vector3d(normal.x(), normal.y(), normal.z()));
    }

    return resultFine;
}

std::string gaalign::GA_LMS_WRAPPER::getName() const {
    if(m_useSteepestDescent) {
        return "GA-LMS (SD)";
    }
    else {
        return "GA-LMS";
    }

}

gaalign::GA_LMS_WRAPPER::GA_LMS_WRAPPER(bool steepestDescent) {
    m_useSteepestDescent = steepestDescent;
}
