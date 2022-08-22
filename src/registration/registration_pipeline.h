//
// Created by Kai on 23.02.2022.
//

#ifndef PCLR_PGA_SRC_REGISTRATION_REGISTRATION_PIPELINE_H_
#define PCLR_PGA_SRC_REGISTRATION_REGISTRATION_PIPELINE_H_

#include <vector>

#include <geometry/point_cloud.h>
#include <geometry/motor.h>

#include <registration/registration_step.h>

namespace gaalign {

    struct PipelineSettings {
        bool visualizeResult = true;
        bool verbose = true;
    };

    /*
     * A registration pipeline is a sequence of registration steps that iteratively register and refine the alignment of two
     * input point clouds. Usually the "source" points are aligned to the "target" points.
     */
    class RegistrationPipeline {
    public:
        // Constructor
        RegistrationPipeline() = default;

        // Add step to pipeline
        void addStep(const RegistrationStep& step);

        // Execute whole pipeline
        gaalign::Motor run(const gaalign::PointCloud &source, const gaalign::PointCloud &target);

        // Print the pipeline as a summary
        void printSummary() const;

        // Getter for the settings
        PipelineSettings& getSettings();

    private:
        // Hold a list of all steps that make up this Pipeline
        std::vector<RegistrationStep> m_steps;

        // Check if the pipeline is valid
        bool isValid() const;

        // Store some settings
        PipelineSettings m_settings;
    };

}

#endif //PCLR_PGA_SRC_REGISTRATION_REGISTRATION_PIPELINE_H_
