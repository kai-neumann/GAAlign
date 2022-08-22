//
// Created by Kai on 13.04.2022.
//

#ifndef GAALIGN_PCL_ICP_WRAPPER_H
#define GAALIGN_PCL_ICP_WRAPPER_H

#include "algorithm_wrapper.h"

namespace gaalign {

    enum PCL_ICP_VARIANT {
        SVD_BASED,                  // Simple Point to Point
        POINT_TO_PLANE,             // Default Point to plane by [Kok-Lim Low 2004] -> See  pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS
        SYMMETRIC_POINT_TO_PLANE    // Symmetric objective proposed by [Rusinkiewicz 2019] -> See  pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS
    };

    /*
     * This is a wrapper for the ICP implementation of PCL (point cloud library)
     */
    class PCLICPWrapper : public gaalign::RegistrationAlgorithmWrapper {
    public:
        PCLICPWrapper(PCL_ICP_VARIANT variant);

        // Initialize the correspondence search. This MUST be called before the actual computation
        gaalign::PointCloud calculateRegistration(const gaalign::PointCloud& source, const gaalign::PointCloud& target, double overlap) const override;

        // Get the name
        std::string getName() const override;

    private:
        PCL_ICP_VARIANT m_variant;
    };
}

#endif //GAALIGN_PCL_ICP_WRAPPER_H
