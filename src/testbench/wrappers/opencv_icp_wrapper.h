//
// Created by Kai on 19.04.2022.
//

#ifndef GAALIGN_OPENCV_ICP_WRAPPER_H
#define GAALIGN_OPENCV_ICP_WRAPPER_H

#include "algorithm_wrapper.h"

namespace gaalign {
    /*
     * This is a wrapper for the ICP implementation of OpenCV
     */
    class OpenCVWrapper : public gaalign::RegistrationAlgorithmWrapper {
    public:
        // Initialize the correspondence search. This MUST be called before the actual computation
        PointCloud calculateRegistration(const PointCloud& source, const PointCloud& target, double overlap) const override;

        // Get the name
        std::string getName() const override;
    };
}


#endif //GAALIGN_OPENCV_ICP_WRAPPER_H
