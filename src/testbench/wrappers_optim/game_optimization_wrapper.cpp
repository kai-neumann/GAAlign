//
// Created by Kai on 04.05.2022.
//

#include "game_optimization_wrapper.h"
#include "testbench/game/MotorEstimationSolver.h"

std::pair<Eigen::Matrix4d, double>
gaalign::GameOptimizationWrapper::calculateRegistration(const std::vector<Correspondence> &correspondences,
                                                        const std::vector<Correspondence> &correspondingNormals) const {
    // Create a motor estimation solver
    game::MotorEstimationSolver solver;

    for(const auto& corr : correspondences) {
        vsr::cga::Pnt a(corr.first.x(), corr.first.y(), corr.first.z());
        vsr::cga::Pnt b(corr.second.x(), corr.second.y(), corr.second.z());

        solver.AddPointCorrespondencesResidualBlock(a, b);
    }

    Mot result = solver.Solve();


    return {};
}

std::string gaalign::GameOptimizationWrapper::getName() const {
    return "GAME";
}

