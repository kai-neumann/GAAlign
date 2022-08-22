//
// Created by Kai on 18.04.2022.
//

#ifndef GAALIGN_MOTOR_ESTIMATION_SSE_H
#define GAALIGN_MOTOR_ESTIMATION_SSE_H

#include <geometry/motor.h>
#include <Eigen/Dense>

namespace gaalign {
    void estimateMotorFromThreeCorrespondencesSSE3(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& correspondences, Motor& outputMotor);
}

#endif //GAALIGN_MOTOR_ESTIMATION_SSE_H
