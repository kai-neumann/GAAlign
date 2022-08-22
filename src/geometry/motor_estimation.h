//
// Created by Kai on 18.01.2022.
//

#ifndef PCLR_PGA_MOTOR_ESTIMATION_H
#define PCLR_PGA_MOTOR_ESTIMATION_H

#include <vector>
#include <unordered_set>
#include <Eigen/Dense>
#include "motor.h"

namespace gaalign {

    // Basic version
    void estimateMotorFromThreeCorrespondences(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& correspondences, Motor& outputMotor);

    void sampleSubIndices(int vectorSize, int groupSize, std::vector<int>& outArray);
}

#endif //PCLR_PGA_MOTOR_ESTIMATION_H
