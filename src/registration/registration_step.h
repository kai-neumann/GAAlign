//
// Created by Kai on 23.02.2022.
//

#ifndef PCLR_PGA_SRC_REGISTRATION_REGISTRATION_STEP_H_
#define PCLR_PGA_SRC_REGISTRATION_REGISTRATION_STEP_H_

#include <optimization/optimizer.h>
#include <correspondence/correspondence_search.h>

namespace gaalign {

    /*
     * A single step in a registration pipeline that consists of a correspondence search algorithm and an optimization algorithm
     */
    class RegistrationStep {
    public:
        // Default constructor
        RegistrationStep() = default;

        // Setters
        void setRepetitions(int repetitions);
        void setOptimizer(const std::shared_ptr<gaalign::Optimizer>& optimizer);
        void setCorrespondenceSearch(const std::shared_ptr<gaalign::CorrespondenceSearch>& search);

        // Getters
        int getRepetitions() const;
        std::shared_ptr<gaalign::Optimizer> getOptimizer() const;
        std::shared_ptr<gaalign::CorrespondenceSearch> getCorrespondenceSearch() const;

    private:
        // Store a pointer to the correspondence search and optimization algorithm
        std::shared_ptr<gaalign::Optimizer> m_optimizer;
        std::shared_ptr<gaalign::CorrespondenceSearch> m_correspondenceSearch;

        // Number of times this step should be repeated (default is only once)
        int m_repetitions = 1;
    };
}

#endif //PCLR_PGA_SRC_REGISTRATION_REGISTRATION_STEP_H_
