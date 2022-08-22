//
// Created by Kai on 27.04.2022.
//

#ifndef GAALIGN_PCL_OPTIMIZATION_WRAPPER_H
#define GAALIGN_PCL_OPTIMIZATION_WRAPPER_H

#include <testbench/wrappers_optim/optimization_wrapper.h>
#include <testbench/wrappers/pcl_icp_wrapper.h>


namespace gaalign {

    class PCLICPOptimizationWrapper : public gaalign::OptimizationAlgorithmWrapper {
    public:
        // Constructor
        PCLICPOptimizationWrapper(gaalign::PCL_ICP_VARIANT variant);

        // Calculate a transformation from known correspondences. Returns the transformation as well as the runtime
        std::pair<Eigen::Matrix4d, double> calculateRegistration(const std::vector<Correspondence> &correspondences, const std::vector<Correspondence> &correspondingNormals) const override;

        // Get the name
        std::string getName() const override;

    private:
        gaalign::PCL_ICP_VARIANT m_variant = gaalign::PCL_ICP_VARIANT::SVD_BASED;
    };
}

#endif //GAALIGN_PCL_OPTIMIZATION_WRAPPER_H
