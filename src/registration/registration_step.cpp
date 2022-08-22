//
// Created by Kai on 23.02.2022.
//

#include "registration_step.h"

void gaalign::RegistrationStep::setRepetitions(int repetitions) {
    m_repetitions = repetitions;
}

int gaalign::RegistrationStep::getRepetitions() const {
    return m_repetitions;
}

void gaalign::RegistrationStep::setOptimizer(const std::shared_ptr<gaalign::Optimizer> &optimizer) {
    m_optimizer = optimizer;
}

void gaalign::RegistrationStep::setCorrespondenceSearch(const std::shared_ptr<gaalign::CorrespondenceSearch> &search) {
    m_correspondenceSearch = search;
}

std::shared_ptr<gaalign::CorrespondenceSearch> gaalign::RegistrationStep::getCorrespondenceSearch() const {
    return m_correspondenceSearch;
}

std::shared_ptr<gaalign::Optimizer> gaalign::RegistrationStep::getOptimizer() const {
    return m_optimizer;
}
