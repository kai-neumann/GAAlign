//
// Created by Kai on 23.02.2022.
//

#include "feature_search.h"

// PCL includes
#include <pcl/filters/voxel_grid.h>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/keypoints/iss_3d.h>

// Teaser for robust matching
#include <correspondence/teaser/matcher.h>

void gaalign::FeatureSearch::init(const PointCloud &source, const PointCloud &target) {
    // Does noting for now. Fixme: Could later maybe initialize detectors and downsample clouds?
}

std::vector<gaalign::Correspondence> gaalign::FeatureSearch::compute(const PointCloud &source, const PointCloud &target) {
    /*
     * The pipeline to calculate correspondences is loosely based on https://stackoverflow.com/questions/30559556/robust-registration-of-two-point-clouds
     */

    std::cout << "Estimating correspondences.." << std::endl;

    // Init the output
    std::vector<std::pair<int, int>> correspondences;

    // Make the sizes relative to the bounding box sizes of source and target
    double maxDimension = fmax(source.getMaxDimension(), target.getMaxDimension());
    double subsampleRadius = m_settings.subSampleRadiusPercentage * maxDimension;
    double subsampleFeatureRadius = m_settings.subSampleFeatureRadiusPercentage * maxDimension;
    double keypointsRadius = m_settings.keypointRadiusPercentage * maxDimension;
    double descriptorRadius = m_settings.descriptorRadiusPercentage * maxDimension;
    //double normalRadius = normalEstimationRadiusPercentage * maxDimension;

    // TODO: Estimate normals for input point cloud, if necessary! -> Save into own PointCloud structure

    // Downsample both point clouds (before detecting keypoints to make calculation faster and more predictable)
    PointCloud sourceSub = source.downsample(subsampleRadius);
    PointCloud targetSub = target.downsample(subsampleRadius);

    // Detect interest points to help the following feature matching
    std::cout << "Detecting keypoints.." << std::endl;
    pcl::PointIndicesConstPtr keypointIndicesSrc = detectKeypoints(sourceSub, keypointsRadius);
    pcl::PointIndicesConstPtr keypointIndicesTarget = detectKeypoints(targetSub, keypointsRadius);

    // Convert the indices to global indices
    pcl::IndicesPtr globalKeypointIndicesSrc(new pcl::Indices());
    globalKeypointIndicesSrc->resize(keypointIndicesSrc->indices.size());
    #pragma omp parallel for
    for (int i = 0; i < keypointIndicesSrc->indices.size(); i++) {
        globalKeypointIndicesSrc->at(i) = sourceSub.getIndex(keypointIndicesSrc->indices.at(i));
    }

    pcl::IndicesPtr globalKeypointIndicesTarget(new pcl::Indices());
    globalKeypointIndicesTarget->resize(keypointIndicesTarget->indices.size());
    #pragma omp parallel for
    for (int i = 0; i < keypointIndicesTarget->indices.size(); i++) {
        globalKeypointIndicesTarget->at(i) = targetSub.getIndex(keypointIndicesTarget->indices.at(i));
    }

    // Convert to Point cloud for visualization
    if(m_settings.visualizeKeypoints) {
        PointCloud keypointsSrcVis;
        for (int i = 0; i < globalKeypointIndicesSrc->size(); i++) {
            keypointsSrcVis.addPoint(source.getPoint(globalKeypointIndicesSrc->at(i)));
        }
        PointCloud keypointsTargetVis;
        for (int i = 0; i < globalKeypointIndicesTarget->size(); i++) {
            keypointsTargetVis.addPoint(target.getPoint(globalKeypointIndicesTarget->at(i)));
        }

        #ifdef ENABLE_VISUALIZATION
        visualizeCorrespondences3D(keypointsSrcVis, keypointsTargetVis, {});
        #endif
    }


    // Initialize the raw correspondence vector
    std::vector<std::pair<int, int>> rawCorrespondences;

    // Subsample the point clouds to a higher detail than the keypoint detection to calculate the descriptors
    PointCloud sourceSubFeature = source.downsample(subsampleFeatureRadius);
    PointCloud targetSubFeature = target.downsample(subsampleFeatureRadius);

    // Use the indices to transfer the keypoints!
    pcl::IndicesPtr subsampledKeypointIndicesSrc(new pcl::Indices());
    // Loop over all points of the downsampled source point cloud
    for(int i=0; i<sourceSubFeature.size(); i++) {
        // Check if the keypoint indices contain the current feature index
        if(std::find(globalKeypointIndicesSrc->begin(), globalKeypointIndicesSrc->end(), sourceSubFeature.getIndex(i)) != globalKeypointIndicesSrc->end()) {
            subsampledKeypointIndicesSrc->push_back(i);
        }
        // TODO: What do we do with keypoints that could not be found?
    }

    pcl::IndicesPtr subsampledKeypointIndicesTarget(new pcl::Indices());
    // Loop over all points of the downsampled source point cloud
    for(int i=0; i<targetSubFeature.size(); i++) {
        // Check if the keypoint indices contain the current feature index
        if(std::find(globalKeypointIndicesTarget->begin(), globalKeypointIndicesTarget->end(), targetSubFeature.getIndex(i)) != globalKeypointIndicesTarget->end()) {
            subsampledKeypointIndicesTarget->push_back(i);
        }
        // TODO: What do we do with keypoints that could not be found?
    }


    // Detect the FPFH features for both point clouds
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr featuresSrc = detectFPFH(sourceSubFeature, subsampledKeypointIndicesSrc, descriptorRadius);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr featuresTarget = detectFPFH(targetSubFeature, subsampledKeypointIndicesTarget, descriptorRadius);

    teaser::PointCloud source_teaser;
    teaser::PointCloud target_teaser;

    for(int i=0; i<subsampledKeypointIndicesSrc->size(); i++) {
        const Eigen::Vector3d& p = sourceSubFeature.getPoint(subsampledKeypointIndicesSrc->at(i));
        source_teaser.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
    }
    for(int i=0; i<subsampledKeypointIndicesTarget->size(); i++) {
        const Eigen::Vector3d& p = targetSubFeature.getPoint(subsampledKeypointIndicesTarget->at(i));
        target_teaser.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
    }

    teaser::Matcher matcher;
    std::vector<std::pair<int, int>> correspondences_teaser = matcher.calculateCorrespondences(
            source_teaser, target_teaser, *featuresSrc, *featuresTarget, false, true, true, 0.95);

    // Transfer features from sub sampled indices to global indices
    rawCorrespondences.resize(correspondences_teaser.size());

    #pragma omp parallel for
    for(int i=0; i<correspondences_teaser.size(); i++) {
        //rawCorrespondences[i] = std::make_pair((int)globalKeypointIndicesSrc->at(correspondences_teaser[i].first), (int)globalKeypointIndicesTarget->at(correspondences_teaser[i].second));
        rawCorrespondences[i] = std::make_pair(sourceSubFeature.getIndex(subsampledKeypointIndicesSrc->at(correspondences_teaser[i].first)),
                                               targetSubFeature.getIndex(subsampledKeypointIndicesTarget->at(correspondences_teaser[i].second)));
    }

    std::cout << "Found " << rawCorrespondences.size() << " raw correspondences" << std::endl;

    // Convert the correspondences to PCL correspondences
    pcl::Correspondences pclCorr;
    for(const auto& corr : rawCorrespondences) {
        pclCorr.emplace_back(corr.first, corr.second, (source.getPoint(corr.first) - target.getPoint(corr.second)).norm());
    }

    // First filter using ransac
    pcl::Correspondences filteredCorrRANSAC;
    {
        std::cout << "Running Rejection Scheme: RANSAC..";
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointNormal> rejectorRANSAC;
        rejectorRANSAC.setInputSource(source.toPCL());
        rejectorRANSAC.setInputTarget(target.toPCL());
        rejectorRANSAC.getRemainingCorrespondences(pclCorr, filteredCorrRANSAC); // Get the correspondences that have not been filtered
        std::cout << " -> " << filteredCorrRANSAC.size() << "/" << pclCorr.size() << " remaining" << std::endl;
    }


    // Then Filter using Poly
    pcl::Correspondences filteredCorrPoly;
    {
        std::cout << "Running Rejection Scheme: Poly..";
        pcl::registration::CorrespondenceRejectorPoly<pcl::PointNormal, pcl::PointNormal> rejectorPoly;
        rejectorPoly.setInputSource(source.toPCL());
        rejectorPoly.setInputTarget(target.toPCL());
        rejectorPoly.getRemainingCorrespondences(filteredCorrRANSAC, filteredCorrPoly); // Get the correspondences that have not been filtered
        std::cout << " -> " << filteredCorrPoly.size() << "/" << filteredCorrRANSAC.size() << " remaining" << std::endl;
    }

    std::cout << filteredCorrPoly.size() << " filtered correspondences remaining!" << std::endl;

    // TODO: Fixme
    if(m_settings.visualizeMatches) {
        std::vector<std::pair<int, int>> corrVis;
        for(const auto& pair : filteredCorrPoly) {
            corrVis.emplace_back(pair.index_query, pair.index_match);
        }

#ifdef ENABLE_VISUALIZATION
        visualizeCorrespondences3D(source, target, corrVis);
#endif
    }

    // Clean up using custom ransac
    /*std::vector<std::pair<int, int>> filteredCorrespondences = verifyCorrespondences(rawCorrespondences, source, target);

    if(m_settings.visualizeMatches) {
        visualizeCorrespondences3D(source, target, filteredCorrespondences);
    }

    // Convert to output data structure
    std::vector<gaalign::Correspondence> outCorrespondences;
    for(const auto& pair : filteredCorrespondences) {
        outCorrespondences.emplace_back(source.getPoint(pair.first), target.getPoint(pair.second));
    }*/

    // Convert to output data structure
    std::vector<gaalign::Correspondence> outCorrespondences;
    for(const auto& pair : filteredCorrPoly) {
        outCorrespondences.emplace_back(source.getPoint(pair.index_query), target.getPoint(pair.index_match));
    }

    return outCorrespondences;
}

std::string gaalign::FeatureSearch::getName() const {
    return "Feature-based Correspondences";
}

gaalign::FeatureSearchSettings &gaalign::FeatureSearch::getSettings() {
    return m_settings;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr
gaalign::FeatureSearch::detectFPFH(const PointCloud &pointCloud, const pcl::IndicesPtr &indices,
                                   double histogramRadius) {
    std::cout << "Converting point cloud for FPFH.." << std::endl;

    // Convert
    pcl::PointCloud<pcl::PointNormal>::Ptr pointsFilteredPtr = pointCloud.toPCL();

    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (pointsFilteredPtr);
    fpfh.setInputNormals (pointsFilteredPtr);

    // Set indices
    fpfh.setIndices(indices);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);

    fpfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (histogramRadius);

    std::cout << "Computing FPFH Features.." << std::endl;
    // Compute the features
    fpfh.compute (*fpfhs);

    // Output
    return fpfhs;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr gaalign::FeatureSearch::extractPoints(const pcl::PointCloud<pcl::PointNormal>::Ptr &pointCloud) {
    // Convert the point cloud and normals into a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ> result;

    // Create pointer (this is done now to avoid costly copying later on)
    pcl::PointCloud<pcl::PointXYZ>::Ptr resultPtr = result.makeShared();

    // Resize to correct size
    resultPtr->resize(pointCloud->size());

    #pragma omp parallel for
    for(int i=0; i<pointCloud->size(); i++) {
        pcl::PointXYZ p;
        p.x = pointCloud->at(i).x;
        p.y = pointCloud->at(i).y;
        p.z = pointCloud->at(i).z;
        resultPtr->at(i) = p;
    }

    return resultPtr;
}

pcl::PointCloud<pcl::Normal>::Ptr gaalign::FeatureSearch::extractNormals(const pcl::PointCloud<pcl::PointNormal>::Ptr &pointCloud) {
    // Convert the point cloud and normals into a PCL point cloud
    pcl::PointCloud<pcl::Normal> result;

    // Create pointer (this is done now to avoid costly copying later on)
    pcl::PointCloud<pcl::Normal>::Ptr resultPtr = result.makeShared();

    // Resize to correct size
    resultPtr->resize(pointCloud->size());

    #pragma omp parallel for
    for(int i=0; i<pointCloud->size(); i++) {
        pcl::Normal n;
        n.normal_x = pointCloud->at(i).normal_x;
        n.normal_y = pointCloud->at(i).normal_y;
        n.normal_z = pointCloud->at(i).normal_z;
        resultPtr->at(i) = n;
    }

    return resultPtr;
}

pcl::PointIndicesConstPtr gaalign::FeatureSearch::detectKeypoints(const PointCloud &pointCloud, double resolution) {
    /*
     * Mainly copied from the PCL documentation https://pointclouds.org/documentation/classpcl_1_1_i_s_s_keypoint3_d.html#details
     */

    // Convert the input to a pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr modelPtr = extractPoints(pointCloud.toPCL());

    // Initialize a search tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    // Initialize the output
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());

    // Calculate the model resolution
    //double resolution = computeCloudResolution(modelPtr);
    //double resolution = 0.01;

    // Initialize the detector
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ, pcl::Normal> iss_detector;
    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (6 * resolution);
    iss_detector.setNonMaxRadius (4 * resolution);
    iss_detector.setThreshold21 (0.975);
    iss_detector.setThreshold32 (0.975);
    iss_detector.setMinNeighbors (5);
    iss_detector.setNumberOfThreads (12);
    iss_detector.setInputCloud (modelPtr);

    // Execute the detector
    iss_detector.compute (*model_keypoints);

    // Print the number of keypoints
    std::cout << "Detected " << iss_detector.getKeypointsIndices()->indices.size() << " keypoints" << std::endl;

    return iss_detector.getKeypointsIndices();
}
