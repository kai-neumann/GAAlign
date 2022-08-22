#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <omp.h>

namespace gaalign {

    struct Motor {
        // A motor in PGA always contains 8 components of even blades. In order those are:
        // 1, e01, e02, e03, e12, e13, e23 and e0123
        std::vector<double> data;

        // Constructor
        Motor() {
            // Initialize with correct size
            data.resize(8);
        }

        void print() {
            // Print the motor
            std::cout << "Motor [";
            for (int i = 0; i < data.size(); i++) {
                std::cout << data[i];
                if (i < data.size() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << "]" << std::endl;
        }

        void normalize();

        void scale(double factor);

        Eigen::Matrix4d toTransformationMatrix() const;

        Motor operator+(const Motor &other) const;
        Motor& operator+=(const Motor &other);

        // Inverse
        Motor inverse() const;

        // Multiplication operator
        static Motor join(const Motor &left, const Motor &right);

        // Get Identity Motor [1, 0, 0, 0, 0, 0, 0, 0]
        static Motor identity();
    };

    Eigen::Vector3d transformPointWithMotor(const Eigen::Vector3d &point, const Motor &motor);
    void transformPointWithMotorInPlace(Eigen::Vector3d &point, const Motor &motor);
}

