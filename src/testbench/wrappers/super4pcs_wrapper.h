//
// Created by Kai on 20.04.2022.
//

#ifndef GAALIGN_SUPER4PCS_WRAPPER_H
#define GAALIGN_SUPER4PCS_WRAPPER_H

#include "algorithm_wrapper.h"
#include <geometry/point_cloud.h>

namespace gaalign {

    /*
     * A wrapper for the Super4PCS algorithm
     */
    class Super4PCSWrapper : public gaalign::RegistrationAlgorithmWrapper {
    public:
        // Constructor taking in the application folder
        explicit Super4PCSWrapper(std::string thirdPartyFolder);

        // Initialize the correspondence search. This MUST be called before the actual computation
        gaalign::PointCloud calculateRegistration(const gaalign::PointCloud& source, const gaalign::PointCloud& target, double overlap) const override;

        // Get the name
        std::string getName() const override;

    private:
        std::string m_executable_path;
        std::string m_folder;

    };
}


#endif //GAALIGN_SUPER4PCS_WRAPPER_H
