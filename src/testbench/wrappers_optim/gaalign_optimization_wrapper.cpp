//
// Created by Kai on 27.04.2022.
//

#include "gaalign_optimization_wrapper.h"
#include "gaalign_cuda_optimization_wrapper.h"


// Time measurement
#include <chrono>

gaalign::GaalignOptimizationWrapper::GaalignOptimizationWrapper(bool useSSE, bool precalculateIndices) {
    m_useSSE = useSSE;
    m_precalculateIndices = precalculateIndices;
}

std::pair<Eigen::Matrix4d, double>
        gaalign::GaalignOptimizationWrapper::calculateRegistration(const std::vector <gaalign::Correspondence> &correspondences,
                                                                                              const std::vector<Correspondence> &correspondingNormals) const {

    gaalign::GradientDescentOptimizer optimizer;
    optimizer.getSettings().enableSSE3 = m_useSSE;
    optimizer.getSettings().precalculateIndices = m_precalculateIndices;

    // Start the time measurement
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Run the algorithm
    gaalign::Motor result = optimizer.optimize(correspondences);

    // Stop the time measurement
    auto end = std::chrono::high_resolution_clock::now();
    double timeMS = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/1000000.0;

    return std::make_pair(result.toTransformationMatrix(), timeMS);
}

std::string gaalign::GaalignOptimizationWrapper::getName() const {
    if(m_precalculateIndices) {
        if(m_useSSE) {
            return "GAAlign SSE3 (PRE)";
        }
        return "GAAlign (PRE)";
    }
    else {
        if(m_useSSE) {
            return "GAAlign SSE3";
        }
        return "GAAlign";
    }


}


