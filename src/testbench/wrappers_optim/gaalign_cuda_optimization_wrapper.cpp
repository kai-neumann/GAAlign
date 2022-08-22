//
// Created by Kai on 29.04.2022.
//

#include "gaalign_cuda_optimization_wrapper.h"
#include <chrono>

gaalign::GaalignCudaOptimizationWrapper::GaalignCudaOptimizationWrapper() {
    // Init the cuda context
    m_optim.init();
}

std::pair<Eigen::Matrix4d, double>
gaalign::GaalignCudaOptimizationWrapper::calculateRegistration(const std::vector<Correspondence> &correspondences,
                                                               const std::vector<Correspondence> &correspondingNormals) const {
    // Start the time measurement
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Run the algorithm
    gaalign::Motor result = m_optim.optimize(correspondences);

    // Stop the time measurement
    auto end = std::chrono::high_resolution_clock::now();
    double timeMS = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/1000000.0;

    return std::make_pair(result.toTransformationMatrix(), timeMS);
}

std::string gaalign::GaalignCudaOptimizationWrapper::getName() const {
    return "GAAlign (CUDA)";
}
