//
// Created by Kai on 23.02.2022.
//

#ifndef PCLR_PGA_SRC_CORRESPONDENCE_NAIVE_CORRESPONDENCE_SEARCH_NAIVE_H_
#define PCLR_PGA_SRC_CORRESPONDENCE_NAIVE_CORRESPONDENCE_SEARCH_NAIVE_H_

#include <correspondence/correspondence_search.h>
#include <geometry/common.h>

namespace gaalign {
    /*
     * A correspondence search implementation that assumes, that the point with index i in the source point cloud always
     * corresponds with the i-th point in the target point cloud.
     */
    class NaiveSearch : public gaalign::CorrespondenceSearch {
    public:
        void init(const PointCloud &source, const PointCloud &target) override;

        std::vector<gaalign::Correspondence> compute(const PointCloud &source, const PointCloud &target) override;

        std::string getName() const override;
    };
}

#endif //PCLR_PGA_SRC_CORRESPONDENCE_NAIVE_CORRESPONDENCE_SEARCH_NAIVE_H_
