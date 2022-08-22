//
// Created by Kai on 13.04.2022.
//

#include "pcl_icp_wrapper.h"

#include <pcl/registration/icp.h>

gaalign::PointCloud gaalign::PCLICPWrapper::calculateRegistration(const gaalign::PointCloud &source, const gaalign::PointCloud &target, double overlap) const {

    if(m_variant == PCL_ICP_VARIANT::SVD_BASED) {
        pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;

        // Set the input source and target
        icp.setInputSource(source.toPCL());
        icp.setInputTarget(target.toPCL());

        double maxDimension = fmax(source.getMaxDimension(), target.getMaxDimension());

        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance(0.025 * maxDimension);
        icp.setMaximumIterations(1000);
        icp.setTransformationEpsilon(1e-9);
        icp.setEuclideanFitnessEpsilon(1);
        icp.setRANSACOutlierRejectionThreshold(1.5);

        // Perform the alignment
        pcl::PointCloud<pcl::PointNormal> cloud_source_registered;
        icp.align(cloud_source_registered);

        // Obtain the transformation that aligned cloud_source to cloud_source_registered
        Eigen::Matrix4f transformation = icp.getFinalTransformation();

        std::cout << "ICP Converged? " << icp.converged_ << std::endl;

        // Convert to new point cloud (in gaalign format)
        gaalign::PointCloud out;
        for (auto &i: cloud_source_registered) {
            out.addPoint(Eigen::Vector3d(i.x, i.y, i.z));
            out.addNormal(Eigen::Vector3d(i.normal_x, i.normal_y, i.normal_z));
        }
        return out;
    }
    else {
        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;

        if(m_variant == PCL_ICP_VARIANT::SYMMETRIC_POINT_TO_PLANE) {
            icp.setUseSymmetricObjective(true);
        }

        // Set the input source and target
        icp.setInputSource(source.toPCL());
        icp.setInputTarget(target.toPCL());

        double maxDimension = fmax(source.getMaxDimension(), target.getMaxDimension());

        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance(0.025 * maxDimension);
        icp.setMaximumIterations(1000);
        icp.setTransformationEpsilon(1e-9);
        icp.setEuclideanFitnessEpsilon(1);
        icp.setRANSACOutlierRejectionThreshold(1.5);

        // Perform the alignment
        pcl::PointCloud<pcl::PointNormal> cloud_source_registered;
        icp.align(cloud_source_registered);

        // Obtain the transformation that aligned cloud_source to cloud_source_registered
        Eigen::Matrix4f transformation = icp.getFinalTransformation();

        std::cout << "ICP Converged? " << icp.converged_ << std::endl;

        // Convert to new point cloud (in gaalign format)
        gaalign::PointCloud out;
        for (auto &i: cloud_source_registered) {
            out.addPoint(Eigen::Vector3d(i.x, i.y, i.z));
            out.addNormal(Eigen::Vector3d(i.normal_x, i.normal_y, i.normal_z));
        }
        return out;
    }
}

std::string gaalign::PCLICPWrapper::getName() const {
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

gaalign::PCLICPWrapper::PCLICPWrapper(gaalign::PCL_ICP_VARIANT variant) {
    m_variant = variant;
}
