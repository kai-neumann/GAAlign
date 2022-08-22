//
// Created by Kai on 29.01.2022.
//

#include "visualize.h"

#include <pcl/visualization/pcl_visualizer.h>

void visualizeCorrespondences3D(const gaalign::PointCloud &source, const gaalign::PointCloud &target,
                                const std::vector<std::pair<int, int>>& correspondences) {
    //
    //  Visualization
    //
    pcl::visualization::PCLVisualizer viewer ("Point Cloud Alignment Visualization");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.setCameraPosition(0, -3, 0, 0, 0, 1, 0);


    // Convert both point clouds to pcl
    pcl::PointCloud<pcl::PointNormal>::Ptr source_pcl = source.toPCL();
    pcl::PointCloud<pcl::PointNormal>::Ptr target_pcl = target.toPCL();

    // Add source
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> source_color_handler (source_pcl, 255, 0, 0);
    viewer.addPointCloud (source_pcl, source_color_handler, "source");

    // Add target
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> target_color_handler (target_pcl, 0, 0, 255);
    viewer.addPointCloud (target_pcl, target_color_handler, "targetz");

    for (std::size_t i = 0; i < correspondences.size (); ++i)
    {
        std::stringstream ss_line;
        ss_line << "correspondence " << i;
        pcl::PointNormal& model_point = source_pcl->at(correspondences[i].first);
        pcl::PointNormal& scene_point = target_pcl->at(correspondences[i].second);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<pcl::PointNormal, pcl::PointNormal> (model_point, scene_point, 255, 0, 255, ss_line.str ());
    }

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }

}