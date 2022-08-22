//
// Created by Kai on 27.04.2022.
//

#ifndef GAALIGN_OPTIMIZATION_WRAPPER_H
#define GAALIGN_OPTIMIZATION_WRAPPER_H

#include "geometry/point_cloud.h"
#include "geometry/common.h"
#include "Eigen/Dense"

namespace gaalign {

    /*
     * The base class for all algorithms that directly take correspondences and calculate a 4x4 transformation matrix
     * This is mainly used for comparing the robustness of algorithms
     */
    class OptimizationAlgorithmWrapper {
    public:
        // Calculate a transformation from known correspondences. Returns the transformation as well as the runtime
        virtual std::pair<Eigen::Matrix4d, double> calculateRegistration(const std::vector<Correspondence> &correspondences, const std::vector<Correspondence> &correspondingNormals) const = 0;

        // Get the name
        virtual std::string getName() const = 0;
    };
}

#endif //GAALIGN_OPTIMIZATION_WRAPPER_H
