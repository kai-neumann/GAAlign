//
// Created by Kai on 15.04.2022.
//

#ifndef GAALIGN_GRADIENT_DESCENT_HYPERPARAMETER_OPTIMIZATION_H
#define GAALIGN_GRADIENT_DESCENT_HYPERPARAMETER_OPTIMIZATION_H

#include <iostream>
#include <vector>
#include "string"

#include <geometry/point_cloud.h>

namespace gaalign {
    // The hyperparameters that get optimized
    struct GaalignHyperParameters {
        // Parameters
        int maxIterations = 100; // [1, 1000]
        int trianglesPerIteration = 100; // [1, 1000]
        double stepSize = 0.05; // (0, 1]
        double minTriangleRelativeEdgeLength = 0.05; // (0,1]
        double minTriangleAngle = 10; // (0, 90]

        // Step sizes for mutation
        double sigma_maxIterations = 10;
        double sigma_triangles = 10;
        double sigma_stepSize = 0.01;
        double sigma_minTriangleRelativeEdgeLength = 0.01;
        double sigma_minTriangleAngle = 2;

        void print() const {
            std::cout << "[i: " << maxIterations << ", t: " << trianglesPerIteration << ", s: " << stepSize << ", e: " << minTriangleRelativeEdgeLength << ", a: " << minTriangleAngle << "]" << std::endl;
        }
    };

    // Evaluate the robustness against noise. Returns the area under the curve (AOC) as well as the average runtime as metrics
    std::pair<double, double> evaluateRobustnessAgainstNoise(const GaalignHyperParameters& params, const std::vector<gaalign::PointCloud>& models, bool visualize);
    std::pair<double, double> evaluateRobustnessAgainstOutliers(const GaalignHyperParameters& params, const std::vector<gaalign::PointCloud>& models, bool visualize);

    // Entry points
    void findOptimalHyperParameters(const std::string& resourceFolder);
    void evaluateRobustnessOfSpecificHyperparameters(const GaalignHyperParameters& params, const std::string& resourceFolder);
}

#endif //GAALIGN_GRADIENT_DESCENT_HYPERPARAMETER_OPTIMIZATION_H
