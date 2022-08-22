//
// Created by Kai on 12.04.2022.
//

#include "gaalign_wrapper.h"

#include <registration/registration_pipeline.h>
#include <registration/registration_step.h>
#include <optimization/gradient_descent/gradient_descent.h>
#include <correspondence/feature/feature_search.h>
#include <correspondence/distance/distance_search.h>
#include <Eigen/Core>

gaalign::PointCloud gaalign::GaalignWrapper::calculateRegistration(const gaalign::PointCloud &source, const gaalign::PointCloud &target, double overlap) const {
    // Initialize the Pipeline
    gaalign::RegistrationPipeline pipeline;
    pipeline.getSettings().verbose = false;
    pipeline.getSettings().visualizeResult = false;

    // -----------------------------------------------------------------------------------------------------------------

    // Add the coarse alignment based on feature-based correspondences and a gradient descent
    gaalign::RegistrationStep coarse;

    std::shared_ptr<gaalign::FeatureSearch> featureSearch = std::make_shared<gaalign::FeatureSearch>();
    featureSearch->getSettings().visualizeKeypoints = false;
    featureSearch->getSettings().visualizeMatches = false;
    coarse.setCorrespondenceSearch(featureSearch);

    std::shared_ptr<gaalign::GradientDescentOptimizer> optimCoarse = std::make_shared<gaalign::GradientDescentOptimizer>();
    //std::shared_ptr<gaalign::GradientDescentOptimizerCUDA> optimCoarse = std::make_shared<gaalign::GradientDescentOptimizerCUDA>();
    optimCoarse->getSettings().maxIterations = 50;
    optimCoarse->getSettings().trianglesPerIteration = 5000;
    optimCoarse->getSettings().stepSize = 0.001;
    optimCoarse->getSettings().verbose = false;
    coarse.setOptimizer(optimCoarse);

    pipeline.addStep(coarse);

    // -----------------------------------------------------------------------------------------------------------------

    // Add the fine alignment that uses distance-based correspondences and a gradient descent
    gaalign::RegistrationStep fine;
    fine.setRepetitions(5);

    std::shared_ptr<gaalign::DistanceSearch> distanceSearch = std::make_shared<gaalign::DistanceSearch>();
    fine.setCorrespondenceSearch(distanceSearch);

    std::shared_ptr<gaalign::GradientDescentOptimizer> optimFine = std::make_shared<gaalign::GradientDescentOptimizer>();
    //std::shared_ptr<gaalign::GradientDescentOptimizerCUDA> optimFine = std::make_shared<gaalign::GradientDescentOptimizerCUDA>();
    optimFine->getSettings().maxIterations = 50;
    optimFine->getSettings().trianglesPerIteration = 5000;
    optimFine->getSettings().stepSize = 0.001;
    fine.setOptimizer(optimFine);

    pipeline.addStep(fine);

    // -----------------------------------------------------------------------------------------------------------------

    // Run the pipeline
    gaalign::Motor resultingMotor = pipeline.run(source, target);

    // Create a working copy of the source, so it can be modified
    gaalign::PointCloud sourceWorkingCopy = source;

    // Apply the motor to the working copy
    sourceWorkingCopy.applyMotor(resultingMotor);

    // Return the transformed point cloud
    return sourceWorkingCopy;
}

std::string gaalign::GaalignWrapper::getName() const {
    return "GAAlign";
}
