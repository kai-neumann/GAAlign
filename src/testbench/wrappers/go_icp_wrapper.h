//
// Created by Kai on 20.04.2022.
//

#ifndef GAALIGN_GO_ICP_WRAPPER_H
#define GAALIGN_GO_ICP_WRAPPER_H

#include "algorithm_wrapper.h"
#include <geometry/point_cloud.h>

namespace gaalign {

    /*
     * The base class for all correspondence search algorithms that takes in two point clouds and returns a list of
     * indices of corresponding points.
     */
    class GOICPWrapper : public gaalign::RegistrationAlgorithmWrapper {
    public:
        // Constructor taking in the application folder
        explicit GOICPWrapper(std::string thirdPartyFolder);

        // Initialize the correspondence search. This MUST be called before the actual computation
        gaalign::PointCloud calculateRegistration(const gaalign::PointCloud& source, const gaalign::PointCloud& target, double overlap) const override;

        // Get the name
        std::string getName() const override;

    private:
        std::string m_executable_path;
    };
}

#endif //GAALIGN_GO_ICP_WRAPPER_H
