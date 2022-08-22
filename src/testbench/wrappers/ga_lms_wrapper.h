//
// Created by Kai on 03.05.2022.
//

#ifndef GAALIGN_GA_LMS_WRAPPER_H
#define GAALIGN_GA_LMS_WRAPPER_H

#include "algorithm_wrapper.h"

namespace gaalign {
    /*
     * The base class for all correspondence search algorithms that takes in two point clouds and returns a list of
     * indices of corresponding points.
     */
    class GA_LMS_WRAPPER : public gaalign::RegistrationAlgorithmWrapper {
    public:
        explicit GA_LMS_WRAPPER(bool steepestDescent);

        // Initialize the correspondence search. This MUST be called before the actual computation
        PointCloud calculateRegistration(const PointCloud& source, const PointCloud& target, double overlap) const override;

        // Get the name
        std::string getName() const override;

    private:
        bool m_useSteepestDescent = false;
    };
}

#endif //GAALIGN_GA_LMS_WRAPPER_H
