//
// Created by Kai on 11.04.2022.
//

#ifndef GAALIGN_SYNTHETIC_DATA_GENERATOR_H
#define GAALIGN_SYNTHETIC_DATA_GENERATOR_H

#include <geometry/point_cloud.h>

std::pair<gaalign::PointCloud, gaalign::PointCloud> slicePointCloud(const gaalign::PointCloud& pointCloud, double overlap);

gaalign::PointCloud disturbWithGaussianNoise(const gaalign::PointCloud& pointCloud, double sigma);

gaalign::PointCloud disturbWithOutliers(const gaalign::PointCloud& pointCloud, double percentage, double maxDimension);

#endif //GAALIGN_SYNTHETIC_DATA_GENERATOR_H
