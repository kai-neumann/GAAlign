#include "iostream"
#include <boost/filesystem.hpp>

// The different testing programs
#include "algorithm_comparison.h"
#include "gradient_descent_hyperparameter_optimization.h"
#include "robustness_tests.h"
#include "performance_evaluation.h"
#include "hyperparameter_evaluation.h"

#include <geometry/motor.h>
#include <geometry/common.h>
#include <chrono>

int main(int argc, char** argv) {
    std::cout << "Starting testbench!" << std::endl;

    // Get the resource folder
    std::string resourceFolder = (boost::filesystem::absolute(argv[0]).parent_path() / "../../resources").string();

    // Compare different algorithms
    compare_algorithms(0.000, resourceFolder, false, true);
    compare_algorithms(0.005, resourceFolder, false, true);

    // Hyper parameter optimization of gradient descent
    //gaalign::findOptimalHyperParameters(resourceFolder);

    compareRobustnessAgainstNoise(resourceFolder, false);
    compareRobustnessAgainstOutliers(resourceFolder, false);

    // Performance tests
    compareRuntimePerformance(resourceFolder, false);

    // Evaluate the hyper parameters
    gaalign::evaluateInfluenceOfHyperparameters(resourceFolder);
}
