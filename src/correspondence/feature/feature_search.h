//
// Created by Kai on 23.02.2022.
//

#ifndef PCLR_PGA_SRC_CORRESPONDENCE_FEATURE_CORRESPONDENCE_SEARCH_FEATURE_H_
#define PCLR_PGA_SRC_CORRESPONDENCE_FEATURE_CORRESPONDENCE_SEARCH_FEATURE_H_

#include <vector>

#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>

#include <correspondence/correspondence_search.h>
#include <geometry/common.h>
#include <geometry/point_cloud.h>

#ifdef ENABLE_VISUALIZATION
#include <visualization/visualize.h>
#endif


namespace gaalign {
    /*
     * Settings struct
     */
    struct FeatureSearchSettings {
        double subSampleRadiusPercentage = 0.002;           // Define the subsample radius for calculating keypoints
        double subSampleFeatureRadiusPercentage = 0.002;    // Define the subsample radius that is used to downsample the point cloud for calculating feature descriptors (should be smaller than the subSampleRadius)
        double keypointRadiusPercentage = 0.005;            // Scale of the keypoint feature detector
        double descriptorRadiusPercentage = 0.05;           // Define the radius of the descriptors
        double normalEstimationRadiusPercentage = 0.02;     // Define the radius of the descriptors
        bool visualizeKeypoints = false;
        bool visualizeMatches = false;
    };

    /*
     * A correspondence search implementation based on ISS keypoints and FPFH features
     */
    class FeatureSearch : public CorrespondenceSearch {
    public:
        void init(const PointCloud &source, const PointCloud &target) override;

        std::vector<gaalign::Correspondence> compute(const PointCloud &source, const PointCloud &target) override;

        std::string getName() const override;

        // Getter for the settings
        FeatureSearchSettings& getSettings();
    private:
        // Hold the algorithms settings
        FeatureSearchSettings m_settings;

        // Calculate FPFH Features for a given point cloud
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr detectFPFH(const PointCloud& pointCloud, const pcl::IndicesPtr& indices, double histogramRadius);

        // Conversion functions to simplify working with PCL clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr extractPoints(const pcl::PointCloud<pcl::PointNormal>::Ptr& pointCloud);
        pcl::PointCloud<pcl::Normal>::Ptr extractNormals(const pcl::PointCloud<pcl::PointNormal>::Ptr& pointCloud);

        // Detect keypoints in a given point cloud
        pcl::PointIndicesConstPtr detectKeypoints(const PointCloud& pointCloud, double resolution);
    };
}

#endif //PCLR_PGA_SRC_CORRESPONDENCE_FEATURE_CORRESPONDENCE_SEARCH_FEATURE_H_
