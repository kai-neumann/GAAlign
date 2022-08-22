//
// Created by Kai on 20.04.2022.
//

#include "go_icp_wrapper.h"

gaalign::GOICPWrapper::GOICPWrapper(std::string thirdPartyFolder) {

}

gaalign::PointCloud gaalign::GOICPWrapper::calculateRegistration(const gaalign::PointCloud &source,
                                                                 const gaalign::PointCloud &target, double overlap) const {
    return gaalign::PointCloud();
}

std::string gaalign::GOICPWrapper::getName() const {
    return "Go-ICP";
}

