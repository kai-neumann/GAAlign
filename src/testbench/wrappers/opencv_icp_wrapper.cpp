//
// Created by Kai on 19.04.2022.
//

#include "opencv_icp_wrapper.h"

#include <opencv2/surface_matching/icp.hpp>
#include <opencv2/surface_matching/ppf_match_3d.hpp>
#include <opencv2/core/eigen.hpp>

gaalign::PointCloud gaalign::OpenCVWrapper::calculateRegistration(const gaalign::PointCloud &source,
                                                                  const gaalign::PointCloud &target, double overlap) const {

    // Convert both point clouds to matrices
    cv::Mat sourceMat(source.size(), 6, CV_32F);
    #pragma omp parallel for
    for(int i=0; i<source.size(); i++) {
        sourceMat.at<float>(i, 0) = (float)source.getPoint(i).x();
        sourceMat.at<float>(i, 1) = (float)source.getPoint(i).y();
        sourceMat.at<float>(i, 2) = (float)source.getPoint(i).z();
        sourceMat.at<float>(i, 3) = (float)source.getNormal(i).x();
        sourceMat.at<float>(i, 4) = (float)source.getNormal(i).y();
        sourceMat.at<float>(i, 5) = (float)source.getNormal(i).z();
    }

    cv::Mat targetMat(target.size(), 6, CV_32F);
    #pragma omp parallel for
    for(int i=0; i<target.size(); i++) {
        targetMat.at<float>(i, 0) = (float)target.getPoint(i).x();
        targetMat.at<float>(i, 1) = (float)target.getPoint(i).y();
        targetMat.at<float>(i, 2) = (float)target.getPoint(i).z();
        targetMat.at<float>(i, 3) = (float)target.getNormal(i).x();
        targetMat.at<float>(i, 4) = (float)target.getNormal(i).y();
        targetMat.at<float>(i, 5) = (float)target.getNormal(i).z();
    }

    // Initialize the ICP Object
    cv::ppf_match_3d::ICP icp(1000, 0.05f, 2.5f, 6, cv::ppf_match_3d::ICP::ICP_SAMPLING_TYPE_UNIFORM, 1);

    // Init the output
    double residual = 0;
    cv::Matx44d pose;

    icp.registerModelToScene(sourceMat, targetMat, residual, pose);

    // Convert the pose to an eigen Matrix
    Eigen::Matrix4d transformation;
    cv::cv2eigen(pose, transformation);

    // Apply the matrix to all points in the point cloud
    gaalign::PointCloud result = source;
    #pragma omp parallel for
    for(int i=0; i<source.size(); i++) {
        // Apply transformation
        Eigen::Vector4d point = transformation * Eigen::Vector4d(source.getPoint(i).x(), source.getPoint(i).y(), source.getPoint(i).z(), 1);
        Eigen::Vector4d normal = transformation * Eigen::Vector4d(source.getNormal(i).x(), source.getNormal(i).y(), source.getNormal(i).z(), 1);

        // Set transformed point / normal to point cloud
        result.setPoint(i, Eigen::Vector3d(point.x(), point.y(), point.z()));
        result.setNormal(i, Eigen::Vector3d(normal.x(), normal.y(), normal.z()));
    }

    return result;
}

std::string gaalign::OpenCVWrapper::getName() const {
    return "OpenCV";
}
