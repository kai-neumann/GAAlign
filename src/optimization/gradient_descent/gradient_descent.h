//
// Created by Kai on 23.02.2022.
//

#ifndef PCLR_PGA_SRC_OPTIMIZATION_GRADIENT_DESCENT_GRADIENT_DESCENT_H_
#define PCLR_PGA_SRC_OPTIMIZATION_GRADIENT_DESCENT_GRADIENT_DESCENT_H_

#include <optimization/optimizer.h>
#include <geometry/common.h>

namespace gaalign {
    /*
     * Settings struct for gradient descent
     */
    struct GradientDescentSettings {
        // Iteration numbers
        int maxIterations = 25;
        int trianglesPerIteration = 1024;

        // Triangle validity checks
        // TODO: Remove
        double minTriangleRelativeEdgeLength = 0.05; // Between 0 and 1
        double minTriangleAngle = 2.5;   // In degrees

        // Step size
        double stepSize = 0.1;

        // This is about 0.5ms seconds faster for ~1000 correspondences, but does not scale well for larger numbers of correspondences
        bool precalculateIndices = false;

        // Momentum
        bool useMomentum = false;
        double momentumStrength = 0.5;

        // Verbosity
        bool verbose = false;
        bool printTiming = false;

        // Enable SSE3 intrinsics for motor estimation
        bool enableSSE3 = true;
    };


    /*
     * An optimizer that uses a stochastical gradient descent (SGD) based on motors (generalized transformations using
     * geometric algebra)
     */
    class GradientDescentOptimizer : public gaalign::Optimizer {
    public:
        // Main entry point for optimization
        Motor optimize(const std::vector<Correspondence> &correspondences) const override;

        // Also expose a name to the outside
        std::string getName() const override;


        // Get the Settings
        GradientDescentSettings& getSettings();
    private:
        GradientDescentSettings m_settings;
    };
}



#endif //PCLR_PGA_SRC_OPTIMIZATION_GRADIENT_DESCENT_GRADIENT_DESCENT_H_
