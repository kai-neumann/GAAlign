//
// Created by Kai on 28.02.2022.
//

#ifndef GAALIGN_COMMON_H
#define GAALIGN_COMMON_H

#include <Eigen/Dense>
#include <utility>

namespace gaalign {
    /*
     * A Pair of points forming a correspondence
     */
    struct Correspondence {
        Eigen::Vector3d first;
        Eigen::Vector3d second;

        Correspondence(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
            first = p1;
            second = p2;
        }

        Correspondence() = default;
    };


    /*
     * A simple triangle struct that hold three Eigen::Vector3d values
     */
    struct Triangle {
        Eigen::Vector3d p1;
        Eigen::Vector3d p2;
        Eigen::Vector3d p3;

        // Constructor based on three points
        Triangle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) {
            this->p1 = p1;
            this->p2 = p2;
            this->p3 = p3;
        }

        // Default constructor
        Triangle() {
            this->p1 = Eigen::Vector3d(0, 0, 0);
            this->p2 = Eigen::Vector3d(0, 0, 0);
            this->p3 = Eigen::Vector3d(0, 0, 0);
        }

        // Check validity based on angles and minimum edge length
        bool isValid(double minEdgeLength, double minAngle) const {
            Eigen::Vector3d AB = p2 - p1;
            Eigen::Vector3d AC = p3 - p1;
            Eigen::Vector3d BC = p3 - p2;

            // Measure the edge length
            if(AB.norm() < minEdgeLength || AC.norm() < minEdgeLength || BC.norm() < minEdgeLength) {
                return false;
            }

            // Convert the minimal angle to radians (approximately)
            double minimalRadians = 0.01745329252*minAngle;

            // Normalize the edges
            AB.normalize();
            AC.normalize();
            BC.normalize();

            // Check the angles
            if(abs(acos(AB.dot(AC))) < minimalRadians || abs(acos((-AB).dot(BC))) < minimalRadians || abs(acos((-AC).dot(-BC))) < minimalRadians) {
                return false;
            }

            return true;
        }

        void applyMotor(const Motor& motor) {
            transformPointWithMotorInPlace(p1, motor);
            transformPointWithMotorInPlace(p2, motor);
            transformPointWithMotorInPlace(p3, motor);
        }
    };

    struct TriangleCorrespondence {
        gaalign::Triangle first;
        gaalign::Triangle second;

        TriangleCorrespondence(gaalign::Triangle  first, gaalign::Triangle  second) : first(std::move(first)), second(std::move(second)) {
        }

        TriangleCorrespondence() {

        }
    };
}

#endif //GAALIGN_COMMON_H
