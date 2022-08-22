//
// Created by Kai on 23.02.2022.
//

#ifndef PCLR_PGA_SRC_CORRESPONDENCE_DISTANCE_CORRESPONDENCE_SEARCH_DISTANCE_H_
#define PCLR_PGA_SRC_CORRESPONDENCE_DISTANCE_CORRESPONDENCE_SEARCH_DISTANCE_H_

#include <pcl/kdtree/kdtree_flann.h>

#include <correspondence/correspondence_search.h>
#include <geometry/common.h>

namespace gaalign {
    /*
     * A settings struct for the DistanceSearch
     */
    struct DistanceSearchSettings {
        double subSampleRadiusPercentage = 0.002;
        double maxOverlapDistancePercentage = 0.01;
        int maximumCorrespondences = 1000;
        bool verbose = true;
    };


    /*
     * A correspondence search algorithm, that always uses the closest point of the target as a corresponding point. A
     * maximum relative distance can be used to only include overlapping areas.
     */
    class DistanceSearch : public gaalign::CorrespondenceSearch {
    public:
        void init(const PointCloud &source, const PointCloud &target) override;

        std::vector<gaalign::Correspondence> compute(const PointCloud &source, const PointCloud &target) override;

        std::string getName() const override;

        DistanceSearchSettings& getSettings();
    private:
        // Settings
        DistanceSearchSettings m_settings;

        // Keep a list of all point indices of the source point cloud which are inside the overlapping area. (precomputed in init())
        std::vector<int> m_overlapIndices;

        // Store the kd tree (precomputed in init())
        pcl::KdTreeFLANN<pcl::PointNormal> m_kdtree;
    };
}

#endif //PCLR_PGA_SRC_CORRESPONDENCE_DISTANCE_CORRESPONDENCE_SEARCH_DISTANCE_H_
