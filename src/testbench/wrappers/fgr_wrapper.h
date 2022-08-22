//
// Created by Kai on 20.04.2022.
//

#ifndef GAALIGN_FGR_WRAPPER_H
#define GAALIGN_FGR_WRAPPER_H

#include "algorithm_wrapper.h"
#include <geometry/point_cloud.h>

namespace gaalign {

    /*
     * A wrapper for the "Fast Global Registration" Algorithm by Qian-Yi Zhou, Jaesik Park, and Vladlen Koltun
     */
    class FGRWrapper : public gaalign::RegistrationAlgorithmWrapper {
    public:
        // Constructor taking in the application folder
        explicit FGRWrapper(std::string thirdPartyFolder);

        // Initialize the correspondence search. This MUST be called before the actual computation
        gaalign::PointCloud calculateRegistration(const gaalign::PointCloud& source, const gaalign::PointCloud& target, double overlap) const override;

        // Get the name
        std::string getName() const override;

    private:
        std::string m_executable_path;
        std::string m_folder;

        void exportPointCloud(const gaalign::PointCloud& pointCloud, const std::string& path) const;
    };
}

#endif //GAALIGN_FGR_WRAPPER_H
