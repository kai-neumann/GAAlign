//
// Created by Kai on 12.04.2022.
//

#ifndef GAALIGN_GAALIGN_WRAPPER_H
#define GAALIGN_GAALIGN_WRAPPER_H

#include "algorithm_wrapper.h"

namespace gaalign {
    /*
     * The base class for all correspondence search algorithms that takes in two point clouds and returns a list of
     * indices of corresponding points.
     */
    class GaalignWrapper : public gaalign::RegistrationAlgorithmWrapper {
    public:
        // Initialize the correspondence search. This MUST be called before the actual computation
        PointCloud calculateRegistration(const PointCloud& source, const PointCloud& target, double overlap) const override;

        // Get the name
        std::string getName() const override;
    };
}

#endif //GAALIGN_GAALIGN_WRAPPER_H
