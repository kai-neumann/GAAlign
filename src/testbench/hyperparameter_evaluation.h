//
// Created by Kai on 02.05.2022.
//

#ifndef GAALIGN_HYPERPARAMETER_EVALUATION_H
#define GAALIGN_HYPERPARAMETER_EVALUATION_H

#include <string>
#include <vector>
#include <geometry/point_cloud.h>

namespace gaalign {
    enum Hyperparameter {
        Iterations,
        StepSize,
        Triangles
    };

    struct HyperparameterResults {
        Hyperparameter paramType;
        std::vector<double> paramValues;
        std::vector<double> rmseValues;
        std::vector<double> runtimeValues;
        double noiseLevel;
    };

    void evaluateInfluenceOfHyperparameters(const std::string& resourceFolder);

    HyperparameterResults evaluateHyperparameter(gaalign::Hyperparameter param, double min, double max, int steps, double noiseStrength, std::vector<gaalign::PointCloud>& models);
}



#endif //GAALIGN_HYPERPARAMETER_EVALUATION_H
