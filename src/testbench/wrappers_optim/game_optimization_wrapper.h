//
// Created by Kai on 04.05.2022.
//

#ifndef GAALIGN_GAME_OPTIMIZATION_WRAPPER_H
#define GAALIGN_GAME_OPTIMIZATION_WRAPPER_H

#include <testbench/wrappers_optim/optimization_wrapper.h>

namespace gaalign {

    class GameOptimizationWrapper : public gaalign::OptimizationAlgorithmWrapper {
    public:
        // Calculate a transformation from known correspondences. Returns the transformation as well as the runtime
        std::pair<Eigen::Matrix4d, double> calculateRegistration(const std::vector<Correspondence> &correspondences,
                                                                 const std::vector<Correspondence> &correspondingNormals) const override;

        // Get the name
        std::string getName() const override;
    };
}


#endif //GAALIGN_GAME_OPTIMIZATION_WRAPPER_H
