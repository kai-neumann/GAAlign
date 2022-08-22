//
// Created by Kai on 29.01.2022.
//

#ifndef PCLR_PGA_VISUALIZE_H
#define PCLR_PGA_VISUALIZE_H

#include <geometry/point_cloud.h>

// Visualize the matches
void visualizeCorrespondences3D(const gaalign::PointCloud &source, const gaalign::PointCloud &target, const std::vector<std::pair<int, int>>& correspondences);

#endif //PCLR_PGA_VISUALIZE_H
