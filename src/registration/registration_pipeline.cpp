//
// Created by Kai on 23.02.2022.
//

#include "registration_pipeline.h"
#include <chrono>

#ifdef ENABLE_VISUALIZATION
#include <visualization/visualize.h>
#endif

void gaalign::RegistrationPipeline::addStep(const gaalign::RegistrationStep &step) {
    // Add the step to the internal list
    m_steps.push_back(step);
}

gaalign::Motor gaalign::RegistrationPipeline::run(const gaalign::PointCloud &source, const gaalign::PointCloud &target) {
    // First check for validity
    if(!isValid()) {
        return Motor::identity();
    }

    // Print summary
    if(m_settings.verbose) printSummary();

    // Create a working copy of the source, so it can be modified
    PointCloud sourceWorkingCopy = source;

    // Initialize the resulting motor
    gaalign::Motor resultingMotor = gaalign::Motor::identity();

    // Loop over all steps one by one
    for(int stepID=0; stepID<m_steps.size(); stepID++) {
        if(m_settings.verbose) std::cout << std::endl << "====================================================================================" << std::endl;
        if(m_settings.verbose) std::cout << "  Executing Step " << stepID + 1 << " (of " << m_steps.size() << "): " <<
                m_steps[stepID].getCorrespondenceSearch()->getName() << " + " << m_steps[stepID].getOptimizer()->getName() << std::endl;
        if(m_settings.verbose) std::cout << "====================================================================================" << std::endl << std::endl;

        // Start timer
        std::chrono::steady_clock::time_point beginStep = std::chrono::steady_clock::now();

        // 1.) Initialize the correspondence search algorithm
        m_steps[stepID].getCorrespondenceSearch()->init(sourceWorkingCopy, target);

        // 2.) For the specified number of repetitions
        for(int repetitionID=0; repetitionID<m_steps[stepID].getRepetitions(); repetitionID++) {
            // 2.1) Run the correspondence search algorithm
            std::vector<Correspondence> correspondences = m_steps[stepID].getCorrespondenceSearch()->compute(sourceWorkingCopy, target);

            // 2.2) Use optimizer to find a motor based on the correspondences
            gaalign::Motor delta = m_steps[stepID].getOptimizer()->optimize(correspondences);

            // 2.3) Apply the delta motor to the source
            sourceWorkingCopy.applyMotor(delta);

            // 2.4) Add the motor to the resulting motor
            resultingMotor = gaalign::Motor::join(delta, resultingMotor);
        }

        // End timer
        auto endStep = std::chrono::high_resolution_clock::now();
        double timeMS = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(endStep - beginStep).count())/1000000.0;

        if(m_settings.verbose) std::cout << "Finished Step " << stepID + 1 << " in " << timeMS << " ms" << std::endl;
    }

    // Visualize result if wished
    #ifdef ENABLE_VISUALIZATION
    if(m_settings.visualizeResult) visualizeCorrespondences3D(sourceWorkingCopy, target, {});
    #endif

    return resultingMotor;
}

void gaalign::RegistrationPipeline::printSummary() const {
    // Sum up how many substeps there are
    int summedRepetitions = 0;
    for(const auto& step : m_steps) {
        summedRepetitions += step.getRepetitions();
    }

    std::cout << std::endl << "====================================================================================" << std::endl;
    std::cout << "  Registration Pipeline with " << m_steps.size() << " steps (" << summedRepetitions << " substeps)" << std::endl;
    std::cout << "------------------------------------------------------------------------------------" << std::endl;
    for(int i=0; i<m_steps.size(); i++) {
        std::cout << "  Step " << i+1 << ": " << "[" << m_steps[i].getCorrespondenceSearch()->getName() <<
                " + " << m_steps[i].getOptimizer()->getName() << "] x " << m_steps[i].getRepetitions() << std::endl;
    }
    std::cout << "====================================================================================" << std::endl << std::endl;
}

bool gaalign::RegistrationPipeline::isValid() const{
    if(m_steps.empty()) {
        std::cerr << "Error during execution of registration pipeline. The pipeline is empty!";
        return false;
    }

    for(int i=0; i<m_steps.size(); i++) {
        // Check the number of repetitions
        if(m_steps[i].getRepetitions() < 1) {
            std::cerr << "Error during execution of registration pipeline. Step " << i+1 << " has an invalid number of repetitions!" << std::endl;
            return false;
        }

        // Check the correspondence search for validity
        if(!m_steps[i].getCorrespondenceSearch()) {
            std::cerr << "Error during execution of registration pipeline. Step " << i+1 << " has an invalid correspondence search algorithm!" << std::endl;
            return false;
        }

        if(!m_steps[i].getOptimizer()) {
            std::cerr << "Error during execution of registration pipeline. Step " << i+1 << " has an invalid optimization algorithm!" << std::endl;
            return false;
        }
    }

    return true;
}

gaalign::PipelineSettings &gaalign::RegistrationPipeline::getSettings() {
    return m_settings;
}
