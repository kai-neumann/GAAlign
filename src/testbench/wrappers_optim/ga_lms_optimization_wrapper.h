//
// Created by Kai on 03.05.2022.
//

#ifndef GAALIGN_GA_LMS_OPTIMIZATION_WRAPPER_H
#define GAALIGN_GA_LMS_OPTIMIZATION_WRAPPER_H

#include "testbench/wrappers_optim/optimization_wrapper.h"

namespace gaalign {

    class GA_LMS_OptimizationWrapper : public gaalign::OptimizationAlgorithmWrapper {
    public:
        GA_LMS_OptimizationWrapper(bool steepestDescent);

        // Calculate a transformation from known correspondences. Returns the transformation as well as the runtime
        std::pair<Eigen::Matrix4d, double> calculateRegistration(const std::vector<Correspondence> &correspondences, const std::vector<Correspondence> &correspondingNormals) const override;

        // Get the name
        std::string getName() const override;

    private:
        bool m_useSteepestDecent = false;
    };
}

#endif //GAALIGN_GA_LMS_OPTIMIZATION_WRAPPER_H
