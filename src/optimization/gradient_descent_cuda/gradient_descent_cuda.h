//
// Created by Kai on 23.02.2022.
//

#ifndef PCLR_PGA_SRC_OPTIMIZATION_GRADIENT_DESCENT_CUDA_GRADIENT_DESCENT_CUDA_H_
#define PCLR_PGA_SRC_OPTIMIZATION_GRADIENT_DESCENT_CUDA_GRADIENT_DESCENT_CUDA_H_


#include <optimization/optimizer.h>
#include <geometry/common.h>

namespace gaalign {
    /*
     * Settings struct for gradient descent
     */
    struct GradientDescentSettingsCUDA {
        // Iteration numbers
        int maxIterations = 25;
        int trianglesPerIteration = 200;

        // Triangle validity checks
        double minTriangleRelativeEdgeLength = 0.05; // Between 0 and 1
        double minTriangleAngle = 2.5;   // In degrees

        // Step size
        double stepSize = 0.1;

        // Also store how many correspondences are supported at maximum -> Should not be larger than 512, to neatly fit into shared memory
        int maxCorrespondences = 256;

        // Verbosity
        bool verbose = false;
        bool printTiming = true;
    };


    /*
     * An optimizer that uses a stochastical gradient descent (SGD) based on motors (generalized transformations using
     * geometric algebra)
     */
    class GradientDescentOptimizerCUDA : public gaalign::Optimizer {
    public:
        // Deconstructor
        ~GradientDescentOptimizerCUDA();

        // Main entry point for optimization
        Motor optimize(const std::vector<Correspondence> &correspondences) const override;

        // Also expose a name to the outside
        std::string getName() const override;

        // Get the Settings
        GradientDescentSettingsCUDA& getSettings();

        // Init Method to preallocate necessary memory.
        // WARNING: CHANGING SETTINGS AFTER CALLING INIT WILL CAUSE UNEXPECTED BEHAVIOUR
        void init();
    private:
        GradientDescentSettingsCUDA m_settings;

        // Store the pointers to the GPU memory, which is allocated and then reused when calling init.
        float* correspondencesGPU;
        unsigned int* indicesGPU;
        float* calculatedMotors;
        float* avgMotor;

        // Tempoaray cpu arrays
        float* correspondenceArrayCPU;

        // Store if it has been initialized
        bool m_initialized = false;
    };
}

#endif //PCLR_PGA_SRC_OPTIMIZATION_GRADIENT_DESCENT_CUDA_GRADIENT_DESCENT_CUDA_H_
