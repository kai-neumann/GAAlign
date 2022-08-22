//
// Created by Kai on 27.04.2022.
//

#ifndef GAALIGN_GAALIGN_OPTIMIZATION_WRAPPER_H
#define GAALIGN_GAALIGN_OPTIMIZATION_WRAPPER_H

#include <testbench/wrappers_optim/optimization_wrapper.h>
#include <optimization/gradient_descent/gradient_descent.h>

namespace gaalign {

    class GaalignOptimizationWrapper : public gaalign::OptimizationAlgorithmWrapper {
    public:
        // Constructor
        GaalignOptimizationWrapper(bool useSSE, bool precalculateIndices);

        // Calculate a transformation from known correspondences. Returns the transformation as well as the runtime
        std::pair<Eigen::Matrix4d, double> calculateRegistration(const std::vector<Correspondence> &correspondences,
                                                                 const std::vector<Correspondence> &correspondingNormals) const override;

        // Get the name
        std::string getName() const override;


    private:
        bool m_useSSE = false;
        bool m_precalculateIndices = false;
    };
}

#endif //GAALIGN_GAALIGN_OPTIMIZATION_WRAPPER_H
