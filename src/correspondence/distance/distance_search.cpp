//
// Created by Kai on 23.02.2022.
//


#include <random>
#include "distance_search.h"

void gaalign::DistanceSearch::init(const PointCloud &source, const PointCloud &target) {
    // Settings
    double maxDimension = fmax(source.getMaxDimension(), target.getMaxDimension());
    double maxOverlapDistance = m_settings.maxOverlapDistancePercentage * maxDimension;

    // Create a kd-tree to easily find the closest point in the target for any input point
    if(m_settings.verbose) std::cout << "Building KD Tree.." << std::endl;
    m_kdtree.setEpsilon(maxOverlapDistance);
    m_kdtree.setInputCloud(target.toPCL());

    // Downsample the source point cloud (to avoid transforming unnecessarily many points per iteration)
    if(m_settings.verbose) std::cout << "Subsampling source point cloud for fine alignment.." << std::endl;
    PointCloud sourceDownsampled = source.downsample(m_settings.subSampleRadiusPercentage * maxDimension);

    // Filter the source point cloud to only include points that are somewhat close to the target (only the overlapping parts)
    // This can improve efficiency of the the sampling
    if(m_settings.verbose) std::cout << "Filtering overlapping points.." << std::endl;

    #pragma omp parallel for
    for(int i=0; i<sourceDownsampled.size(); i++) {
        // Init the output arrays for the nearest neighbour search
        std::vector<int> pointIdx;
        std::vector<float> pointDistances;

        // Convert the search point to a pcl point
        const Eigen::Vector3d& p = sourceDownsampled.getPoint(i);
        pcl::PointNormal point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();

        // If there is at least one point in the search radius: Add that point to the overlap cloud
        if(m_kdtree.radiusSearch (point, maxOverlapDistance, pointIdx, pointDistances) > 0) {
            #pragma omp critical
            {
                m_overlapIndices.push_back(sourceDownsampled.getIndex(i));
            }
        }
    }

    if(m_settings.verbose) std::cout << "Found " << m_overlapIndices.size() << " overlapping points (out of " << sourceDownsampled.size() << " points)" << std::endl;
    if(m_settings.verbose) std::cout << "Estimated Overlap: " << (((double)m_overlapIndices.size())/((double)sourceDownsampled.size()))*100.0 << "%" << std::endl;
}

std::vector<gaalign::Correspondence> gaalign::DistanceSearch::compute(const PointCloud &source, const PointCloud &target) {
    // Init the output
    std::vector<Correspondence> correspondences;

    // Initialize the array of indices used for building correspondences
    std::vector<int> tempIndices;

    // If there are less available points than allowed in the settings: Use all of them
    if(m_overlapIndices.size() < m_settings.maximumCorrespondences) {
        tempIndices = m_overlapIndices;
    }
    // Else: Sample the specified number
    else {
        // Sample N indices
        std::sample(m_overlapIndices.begin(), m_overlapIndices.end(), std::back_inserter(tempIndices),
                    m_settings.maximumCorrespondences, std::mt19937{std::random_device{}()});
    }

    #pragma omp parallel for
    for(int i=0; i<tempIndices.size(); i++) {
        // Init the output arrays for the nearest neighbour search
        std::vector<int> pointIdx;
        std::vector<float> pointDistances;

        // Access once to avoid multiple reads
        const Eigen::Vector3d& point = source.getPoint(tempIndices[i]);

        pcl::PointNormal pointPCL;
        pointPCL.x = point.x();
        pointPCL.y = point.y();
        pointPCL.z = point.z();

        // Search for the nearest neighbour (within epsilon)
        if(m_kdtree.nearestKSearch(pointPCL, 1, pointIdx, pointDistances) > 0) {
            #pragma omp critical
            {
                correspondences.emplace_back(point, target.getPoint(pointIdx[0]));
            }

        }
    }

    return correspondences;
}

std::string gaalign::DistanceSearch::getName() const {
    return "Distance-based Correspondences";
}

gaalign::DistanceSearchSettings &gaalign::DistanceSearch::getSettings() {
    return m_settings;
}
