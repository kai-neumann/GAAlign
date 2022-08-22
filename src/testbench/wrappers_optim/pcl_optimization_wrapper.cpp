//
// Created by Kai on 27.04.2022.
//

#include "pcl_optimization_wrapper.h"

#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_symmetric_point_to_plane_lls.h>
#include <chrono>

gaalign::PCLICPOptimizationWrapper::PCLICPOptimizationWrapper(gaalign::PCL_ICP_VARIANT variant) {
    m_variant = variant;
}

std::pair<Eigen::Matrix4d, double>
gaalign::PCLICPOptimizationWrapper::calculateRegistration(const std::vector<Correspondence> &correspondences,
                                                          const std::vector<Correspondence> &correspondingNormals) const {
    // Convert the correspondences into two pcl Point clouds
    pcl::PointCloud<pcl::PointNormal> sourceCorr;
    pcl::PointCloud<pcl::PointNormal> targetCorr;

    // Resize
    sourceCorr.resize(correspondences.size());
    targetCorr.resize(correspondences.size());

    #pragma omp parallel for
    for(int i=0; i<correspondences.size(); i++) {
        // Source
        pcl::PointNormal pn1;
        pn1.x = correspondences[i].first.x();
        pn1.y = correspondences[i].first.y();
        pn1.z = correspondences[i].first.z();
        pn1.normal_x = correspondingNormals[i].first.x();
        pn1.normal_y = correspondingNormals[i].first.y();
        pn1.normal_z = correspondingNormals[i].first.z();
        sourceCorr[i] = pn1;

        // Target
        pcl::PointNormal pn2;
        pn2.x = correspondences[i].second.x();
        pn2.y = correspondences[i].second.y();
        pn2.z = correspondences[i].second.z();
        pn2.normal_x = correspondingNormals[i].second.x();
        pn2.normal_y = correspondingNormals[i].second.y();
        pn2.normal_z = correspondingNormals[i].second.z();
        targetCorr[i] = pn2;
    }

    // Init the result
    Eigen::Matrix4d result;
    double runtime = 0;

    if(m_variant == PCL_ICP_VARIANT::SVD_BASED) {
        // Init the registration object
        pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal, double> est;

        // Start the time measurement
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Compute transformation
        est.estimateRigidTransformation(sourceCorr, targetCorr, result);

        // Stop the time measurement
        auto end = std::chrono::high_resolution_clock::now();
        runtime = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/1000000.0;
    }
    else if(m_variant == PCL_ICP_VARIANT::POINT_TO_PLANE) {
        // Init the registration object
        pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal, double> est;

        // Start the time measurement
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Compute transformation
        est.estimateRigidTransformation(sourceCorr, targetCorr, result);

        // Stop the time measurement
        auto end = std::chrono::high_resolution_clock::now();
        runtime = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/1000000.0;
    }
    else {
        // Init the registration object
        pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal, double> est;

        // Start the time measurement
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Compute transformation
        est.estimateRigidTransformation(sourceCorr, targetCorr, result);

        // Stop the time measurement
        auto end = std::chrono::high_resolution_clock::now();
        runtime = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/1000000.0;
    }



    return std::make_pair(result, runtime);
}

std::string gaalign::PCLICPOptimizationWrapper::getName() const {
    if(m_variant == gaalign::PCL_ICP_VARIANT::SVD_BASED) {
        return "PCL (Point2Point)";
    }
    else if(m_variant == gaalign::PCL_ICP_VARIANT::POINT_TO_PLANE) {
        return "PCL (Point2Plane)";
    }
        // Symmetric point to plane
    else {
        return "PCL (Symm. Point2Plane)";
    }
}
