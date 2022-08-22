//
// Created by Kai on 23.02.2022.
//

#ifndef PCLR_PGA_SRC_CORRESPONDENCE_CORRESPONDENCE_SEARCH_H_
#define PCLR_PGA_SRC_CORRESPONDENCE_CORRESPONDENCE_SEARCH_H_

#include <vector>

#include <geometry/point_cloud.h>
#include <geometry/common.h>

namespace gaalign {

    /*
     * The base class for all correspondence search algorithms that takes in two point clouds and returns a list of
     * indices of corresponding points.
     */
    class CorrespondenceSearch {
    public:
        // Initialize the correspondence search. This MUST be called before the actual computation
        virtual void init(const PointCloud& source, const PointCloud& target) = 0;

        // Run the computation based on both point clouds
        virtual std::vector<gaalign::Correspondence> compute(const PointCloud& source, const PointCloud& target) = 0;

        // Get the name
        virtual std::string getName() const = 0;
    };
}

#endif //PCLR_PGA_SRC_CORRESPONDENCE_CORRESPONDENCE_SEARCH_H_
