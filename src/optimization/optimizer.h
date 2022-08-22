//
// Created by Kai on 23.02.2022.
//

#ifndef PCLR_PGA_SRC_OPTIMIZATION_OPTIMIZER_H_
#define PCLR_PGA_SRC_OPTIMIZATION_OPTIMIZER_H_

#include <geometry/motor.h>
#include <geometry/common.h>

namespace gaalign {

    /*
     * This is the base class for any optimization algorithm (e.g. Gradient descent) that takes in a number of correspondences
     * and iteratively calculates a transformation (called Motor in PGA)
     */
    class Optimizer {
    public:
        // Main entry point for optimization
        virtual Motor optimize(const std::vector<Correspondence>& correspondences) const = 0;

        // Also expose a name to the outside
        virtual std::string getName() const = 0;
    };
}

#endif //PCLR_PGA_SRC_OPTIMIZATION_OPTIMIZER_H_
