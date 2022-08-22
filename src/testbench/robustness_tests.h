//
// Created by Kai on 27.04.2022.
//

#ifndef GAALIGN_ROBUSTNESS_TESTS_H
#define GAALIGN_ROBUSTNESS_TESTS_H

#include <string>
#include <vector>
#include <geometry/point_cloud.h>
#include <iostream>

void compareRobustnessAgainstNoise(const std::string& resourceFolder, bool visualize);

void compareRobustnessAgainstOutliers(const std::string& resourceFolder, bool visualize);

#endif //GAALIGN_ROBUSTNESS_TESTS_H
