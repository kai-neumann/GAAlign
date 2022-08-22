//
// Created by Kai on 11.04.2022.
//

#ifndef GAALIGN_ALGORITHM_WRAPPER_H
#define GAALIGN_ALGORITHM_WRAPPER_H

#include "geometry/point_cloud.h"

namespace gaalign {

    /*
     * The base class for all correspondence search algorithms that takes in two point clouds and returns a list of
     * indices of corresponding points.
     */
    class RegistrationAlgorithmWrapper {
    public:
        // Initialize the correspondence search. This MUST be called before the actual computation
        virtual gaalign::PointCloud calculateRegistration(const gaalign::PointCloud& source, const gaalign::PointCloud& target, double overlap) const = 0;

        // Get the name
        virtual std::string getName() const = 0;
    };
}

#endif //GAALIGN_ALGORITHM_WRAPPER_H
