#line 1 "D:/Development/GAAlign/src/geometry/motor.cpg"
#include "D:/Development/GAAlign/src/geometry/motor.h"

Eigen::Vector3d gaalign::transformPointWithMotor(const Eigen::Vector3d& point, const Motor& motor) {
    const double* motorArr = motor.data.data();
    double pointArr[4] = {-point.x(), point.y(), -point.z(), 1.0};
    double outPoint[4] = {0};



#line 16 "D:/Development/GAAlign/src/geometry/motor.cpg"
#include <math.h>
//#pragma gpc multivector temp
double temp[7];
//#pragma gpc multivector transformedPoint
double transformedPoint[4];

//#pragma gpc multivector_component temp e1 temp[0]
temp[0] = (-(motorArr[6] * pointArr[3])) * motorArr[0] + (-(motorArr[5] * pointArr[3] * (-motorArr[4]))) + (-((-(motorArr[4] * pointArr[3])) * (-motorArr[5]))) + (-(motorArr[0] * pointArr[3] * (-motorArr[6])));
//#pragma gpc multivector_component temp e2 temp[1]
temp[1] = (-(motorArr[6] * pointArr[3])) * (-motorArr[4]) + motorArr[5] * pointArr[3] * motorArr[0] + (-((-(motorArr[4] * pointArr[3])) * (-motorArr[6]))) + motorArr[0] * pointArr[3] * (-motorArr[5]);
//#pragma gpc multivector_component temp e3 temp[2]
temp[2] = (-(motorArr[6] * pointArr[3])) * (-motorArr[5]) + motorArr[5] * pointArr[3] * (-motorArr[6]) + (-(motorArr[4] * pointArr[3])) * motorArr[0] + (-(motorArr[0] * pointArr[3] * (-motorArr[4])));
//#pragma gpc multivector_component temp e0^e1^e2 temp[3]
temp[3] = ((-(motorArr[4] * pointArr[2])) + (-(motorArr[5] * pointArr[1])) + (-(motorArr[6] * pointArr[0])) + (-(motorArr[7] * pointArr[3]))) * (-motorArr[4]) + (-((-(motorArr[6] * pointArr[3])) * (-motorArr[2]))) + motorArr[5] * pointArr[3] * (-motorArr[1]) + (-((-(motorArr[4] * pointArr[3])) * motorArr[7])) + (motorArr[0] * pointArr[2] + motorArr[3] * pointArr[3] + (-(motorArr[5] * pointArr[0])) + motorArr[6] * pointArr[1]) * motorArr[0] + (-((motorArr[0] * pointArr[1] + (-(motorArr[2] * pointArr[3])) + motorArr[4] * pointArr[0] + (-(motorArr[6] * pointArr[2]))) * (-motorArr[6]))) + (motorArr[0] * pointArr[0] + motorArr[1] * pointArr[3] + (-(motorArr[4] * pointArr[1])) + motorArr[5] * pointArr[2]) * (-motorArr[5]) + (-(motorArr[0] * pointArr[3] * (-motorArr[3])));
//#pragma gpc multivector_component temp e0^e1^e3 temp[4]
temp[4] = ((-(motorArr[4] * pointArr[2])) + (-(motorArr[5] * pointArr[1])) + (-(motorArr[6] * pointArr[0])) + (-(motorArr[7] * pointArr[3]))) * (-motorArr[5]) + (-((-(motorArr[6] * pointArr[3])) * (-motorArr[3]))) + motorArr[5] * pointArr[3] * motorArr[7] + (-(motorArr[4] * pointArr[3])) * (-motorArr[1]) + (motorArr[0] * pointArr[2] + motorArr[3] * pointArr[3] + (-(motorArr[5] * pointArr[0])) + motorArr[6] * pointArr[1]) * (-motorArr[6]) + (motorArr[0] * pointArr[1] + (-(motorArr[2] * pointArr[3])) + motorArr[4] * pointArr[0] + (-(motorArr[6] * pointArr[2]))) * motorArr[0] + (-((motorArr[0] * pointArr[0] + motorArr[1] * pointArr[3] + (-(motorArr[4] * pointArr[1])) + motorArr[5] * pointArr[2]) * (-motorArr[4]))) + motorArr[0] * pointArr[3] * (-motorArr[2]);
//#pragma gpc multivector_component temp e0^e2^e3 temp[5]
temp[5] = ((-(motorArr[4] * pointArr[2])) + (-(motorArr[5] * pointArr[1])) + (-(motorArr[6] * pointArr[0])) + (-(motorArr[7] * pointArr[3]))) * (-motorArr[6]) + (-((-(motorArr[6] * pointArr[3])) * motorArr[7])) + (-(motorArr[5] * pointArr[3] * (-motorArr[3]))) + (-(motorArr[4] * pointArr[3])) * (-motorArr[2]) + (-((motorArr[0] * pointArr[2] + motorArr[3] * pointArr[3] + (-(motorArr[5] * pointArr[0])) + motorArr[6] * pointArr[1]) * (-motorArr[5]))) + (motorArr[0] * pointArr[1] + (-(motorArr[2] * pointArr[3])) + motorArr[4] * pointArr[0] + (-(motorArr[6] * pointArr[2]))) * (-motorArr[4]) + (motorArr[0] * pointArr[0] + motorArr[1] * pointArr[3] + (-(motorArr[4] * pointArr[1])) + motorArr[5] * pointArr[2]) * motorArr[0] + (-(motorArr[0] * pointArr[3] * (-motorArr[1])));
//#pragma gpc multivector_component temp e1^e2^e3 temp[6]
temp[6] = (-(motorArr[6] * pointArr[3])) * (-motorArr[6]) + (-(motorArr[5] * pointArr[3] * (-motorArr[5]))) + (-(motorArr[4] * pointArr[3])) * (-motorArr[4]) + motorArr[0] * pointArr[3] * motorArr[0];
//#pragma gpc multivector_component transformedPoint e0^e1^e2 transformedPoint[0]
transformedPoint[0] = temp[3] * sqrtf(fabs(temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6]))))) / sqrtf(fabs((temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6])))) * (temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6]))))));
//#pragma gpc multivector_component transformedPoint e0^e1^e3 transformedPoint[1]
transformedPoint[1] = temp[4] * sqrtf(fabs(temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6]))))) / sqrtf(fabs((temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6])))) * (temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6]))))));
//#pragma gpc multivector_component transformedPoint e0^e2^e3 transformedPoint[2]
transformedPoint[2] = temp[5] * sqrtf(fabs(temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6]))))) / sqrtf(fabs((temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6])))) * (temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6]))))));
//#pragma gpc multivector_component transformedPoint e1^e2^e3 transformedPoint[3]
transformedPoint[3] = temp[6] * sqrtf(fabs(temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6]))))) / sqrtf(fabs((temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6])))) * (temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2] + (-(temp[6] * (-temp[6]))))));

#line 22 "D:/Development/GAAlign/src/geometry/motor.cpg"


outPoint[0] = transformedPoint[2];
outPoint[1] = transformedPoint[1];
outPoint[2] = transformedPoint[0];
outPoint[3] = transformedPoint[3];



#line 27 "D:/Development/GAAlign/src/geometry/motor.cpg"

    // Return the point as vector
    return Eigen::Vector3d(-outPoint[0], outPoint[1], -outPoint[2]);

}

void gaalign::transformPointWithMotorInPlace(Eigen::Vector3d &point, const gaalign::Motor &motor) {
    const double* motorArr = motor.data.data();
    double pointArr[4] = {-point.x(), point.y(), -point.z(), 1.0};
    double outPoint[4] = {0};



#line 46 "D:/Development/GAAlign/src/geometry/motor.cpg"
//#pragma gpc multivector temp1
double temp1[4];

//#pragma gpc multivector_component temp1 e0^e1^e2 temp1[0]
temp1[0] = ((-(motorArr[4] * pointArr[2])) + (-(motorArr[5] * pointArr[1])) + (-(motorArr[6] * pointArr[0])) + (-(motorArr[7] * pointArr[3]))) * (-motorArr[4]) + (-((-(motorArr[6] * pointArr[3])) * (-motorArr[2]))) + motorArr[5] * pointArr[3] * (-motorArr[1]) + (-((-(motorArr[4] * pointArr[3])) * motorArr[7])) + (motorArr[0] * pointArr[2] + motorArr[3] * pointArr[3] + (-(motorArr[5] * pointArr[0])) + motorArr[6] * pointArr[1]) * motorArr[0] + (-((motorArr[0] * pointArr[1] + (-(motorArr[2] * pointArr[3])) + motorArr[4] * pointArr[0] + (-(motorArr[6] * pointArr[2]))) * (-motorArr[6]))) + (motorArr[0] * pointArr[0] + motorArr[1] * pointArr[3] + (-(motorArr[4] * pointArr[1])) + motorArr[5] * pointArr[2]) * (-motorArr[5]) + (-(motorArr[0] * pointArr[3] * (-motorArr[3])));
//#pragma gpc multivector_component temp1 e0^e1^e3 temp1[1]
temp1[1] = ((-(motorArr[4] * pointArr[2])) + (-(motorArr[5] * pointArr[1])) + (-(motorArr[6] * pointArr[0])) + (-(motorArr[7] * pointArr[3]))) * (-motorArr[5]) + (-((-(motorArr[6] * pointArr[3])) * (-motorArr[3]))) + motorArr[5] * pointArr[3] * motorArr[7] + (-(motorArr[4] * pointArr[3])) * (-motorArr[1]) + (motorArr[0] * pointArr[2] + motorArr[3] * pointArr[3] + (-(motorArr[5] * pointArr[0])) + motorArr[6] * pointArr[1]) * (-motorArr[6]) + (motorArr[0] * pointArr[1] + (-(motorArr[2] * pointArr[3])) + motorArr[4] * pointArr[0] + (-(motorArr[6] * pointArr[2]))) * motorArr[0] + (-((motorArr[0] * pointArr[0] + motorArr[1] * pointArr[3] + (-(motorArr[4] * pointArr[1])) + motorArr[5] * pointArr[2]) * (-motorArr[4]))) + motorArr[0] * pointArr[3] * (-motorArr[2]);
//#pragma gpc multivector_component temp1 e0^e2^e3 temp1[2]
temp1[2] = ((-(motorArr[4] * pointArr[2])) + (-(motorArr[5] * pointArr[1])) + (-(motorArr[6] * pointArr[0])) + (-(motorArr[7] * pointArr[3]))) * (-motorArr[6]) + (-((-(motorArr[6] * pointArr[3])) * motorArr[7])) + (-(motorArr[5] * pointArr[3] * (-motorArr[3]))) + (-(motorArr[4] * pointArr[3])) * (-motorArr[2]) + (-((motorArr[0] * pointArr[2] + motorArr[3] * pointArr[3] + (-(motorArr[5] * pointArr[0])) + motorArr[6] * pointArr[1]) * (-motorArr[5]))) + (motorArr[0] * pointArr[1] + (-(motorArr[2] * pointArr[3])) + motorArr[4] * pointArr[0] + (-(motorArr[6] * pointArr[2]))) * (-motorArr[4]) + (motorArr[0] * pointArr[0] + motorArr[1] * pointArr[3] + (-(motorArr[4] * pointArr[1])) + motorArr[5] * pointArr[2]) * motorArr[0] + (-(motorArr[0] * pointArr[3] * (-motorArr[1])));
//#pragma gpc multivector_component temp1 e1^e2^e3 temp1[3]
temp1[3] = (-(motorArr[6] * pointArr[3])) * (-motorArr[6]) + (-(motorArr[5] * pointArr[3] * (-motorArr[5]))) + (-(motorArr[4] * pointArr[3])) * (-motorArr[4]) + motorArr[0] * pointArr[3] * motorArr[0];

#line 50 "D:/Development/GAAlign/src/geometry/motor.cpg"


outPoint[0] = temp1[2];
outPoint[1] = temp1[1];
outPoint[2] = temp1[0];
outPoint[3] = temp1[3];



#line 55 "D:/Development/GAAlign/src/geometry/motor.cpg"

    // Return the point as vector
    point.x() = -outPoint[0] / outPoint[3];
    point.y() = outPoint[1] / outPoint[3];
    point.z() = -outPoint[2] / outPoint[3];

}

void gaalign::Motor::normalize() {
    for(int i=1; i<data.size(); i++) {
        data[i] = data[i] / data[0];
    }
    data[0] = 1.0;
}

void gaalign::Motor::scale(double factor) {
    // If this is a pure bi vector (which can be created after taking the logarithm) -> Multiply all components
    if(abs(data[0]) < 1e-8) {
        for(int i=0; i<data.size(); i++) {
            data[i] = data[i]*factor;
        }
    }
    // Else we need to be careful with the homogenous component
    else {
        // Scale all components linearily except the one at index 0
        for(int i=1; i<data.size(); i++) {
            data[i] = data[i]*factor;
        }

        // Interpolate the component at 0
        data[0] = (data[0] - 1.0) * factor + 1.0;
    }


}

gaalign::Motor gaalign::Motor::identity() {
    Motor m;
    m.data[0] = 1;
    return m;
}

gaalign::Motor gaalign::Motor::join(const Motor &left, const Motor &right) {
    // Get both raw data pointers
    const double* motor1Arr = left.data.data();
    const double* motor2Arr = right.data.data();

    // Init output
    double outMotor[8] = {0.0};



#line 111 "D:/Development/GAAlign/src/geometry/motor.cpg"
#include <math.h>
//#pragma gpc multivector combined
double combined[8];
//#pragma gpc multivector motor_norm
double motor_norm;
//#pragma gpc multivector normed_motor
double normed_motor[8];

//#pragma gpc multivector_component combined 1.0 combined[0]
combined[0] = motor1Arr[0] * motor2Arr[0] + (-(motor1Arr[4] * motor2Arr[4])) + (-(motor1Arr[5] * motor2Arr[5])) + (-(motor1Arr[6] * motor2Arr[6]));
//#pragma gpc multivector_component combined e0^e1 combined[1]
combined[1] = motor1Arr[0] * motor2Arr[1] + motor1Arr[1] * motor2Arr[0] + (-(motor1Arr[2] * motor2Arr[4])) + (-(motor1Arr[3] * motor2Arr[5])) + motor1Arr[4] * motor2Arr[2] + motor1Arr[5] * motor2Arr[3] + (-(motor1Arr[6] * motor2Arr[7])) + (-(motor1Arr[7] * motor2Arr[6]));
//#pragma gpc multivector_component combined e0^e2 combined[2]
combined[2] = motor1Arr[0] * motor2Arr[2] + motor1Arr[1] * motor2Arr[4] + motor1Arr[2] * motor2Arr[0] + (-(motor1Arr[3] * motor2Arr[6])) + (-(motor1Arr[4] * motor2Arr[1])) + motor1Arr[5] * motor2Arr[7] + motor1Arr[6] * motor2Arr[3] + motor1Arr[7] * motor2Arr[5];
//#pragma gpc multivector_component combined e0^e3 combined[3]
combined[3] = motor1Arr[0] * motor2Arr[3] + motor1Arr[1] * motor2Arr[5] + motor1Arr[2] * motor2Arr[6] + motor1Arr[3] * motor2Arr[0] + (-(motor1Arr[4] * motor2Arr[7])) + (-(motor1Arr[5] * motor2Arr[1])) + (-(motor1Arr[6] * motor2Arr[2])) + (-(motor1Arr[7] * motor2Arr[4]));
//#pragma gpc multivector_component combined e1^e2 combined[4]
combined[4] = motor1Arr[0] * motor2Arr[4] + motor1Arr[4] * motor2Arr[0] + (-(motor1Arr[5] * motor2Arr[6])) + motor1Arr[6] * motor2Arr[5];
//#pragma gpc multivector_component combined e1^e3 combined[5]
combined[5] = motor1Arr[0] * motor2Arr[5] + motor1Arr[4] * motor2Arr[6] + motor1Arr[5] * motor2Arr[0] + (-(motor1Arr[6] * motor2Arr[4]));
//#pragma gpc multivector_component combined e2^e3 combined[6]
combined[6] = motor1Arr[0] * motor2Arr[6] + (-(motor1Arr[4] * motor2Arr[5])) + motor1Arr[5] * motor2Arr[4] + motor1Arr[6] * motor2Arr[0];
//#pragma gpc multivector_component combined e0^e1^e2^e3 combined[7]
combined[7] = motor1Arr[0] * motor2Arr[7] + motor1Arr[1] * motor2Arr[6] + (-(motor1Arr[2] * motor2Arr[5])) + motor1Arr[3] * motor2Arr[4] + motor1Arr[4] * motor2Arr[3] + (-(motor1Arr[5] * motor2Arr[2])) + motor1Arr[6] * motor2Arr[1] + motor1Arr[7] * motor2Arr[0];
//#pragma gpc multivector_component motor_norm 1.0 motor_norm
motor_norm = sqrtf(fabs(combined[0] * combined[0] + (-(combined[4] * (-combined[4]))) + (-(combined[5] * (-combined[5]))) + (-(combined[6] * (-combined[6])))));
//#pragma gpc multivector_component normed_motor 1.0 normed_motor[0]
normed_motor[0] = combined[0] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component normed_motor e0^e1 normed_motor[1]
normed_motor[1] = combined[1] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component normed_motor e0^e2 normed_motor[2]
normed_motor[2] = combined[2] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component normed_motor e0^e3 normed_motor[3]
normed_motor[3] = combined[3] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component normed_motor e1^e2 normed_motor[4]
normed_motor[4] = combined[4] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component normed_motor e1^e3 normed_motor[5]
normed_motor[5] = combined[5] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component normed_motor e2^e3 normed_motor[6]
normed_motor[6] = combined[6] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component normed_motor e0^e1^e2^e3 normed_motor[7]
normed_motor[7] = combined[7] * motor_norm / (motor_norm * motor_norm);

#line 120 "D:/Development/GAAlign/src/geometry/motor.cpg"


outMotor[0] = normed_motor[0];
outMotor[1] = normed_motor[1];
outMotor[2] = normed_motor[2];
outMotor[3] = normed_motor[3];
outMotor[4] = normed_motor[4];
outMotor[5] = normed_motor[5];
outMotor[6] = normed_motor[6];
outMotor[7] = normed_motor[7];



#line 125 "D:/Development/GAAlign/src/geometry/motor.cpg"

    // Copy data to Motor struct
    Motor m;
    m.data = std::vector<double>(outMotor, outMotor + (sizeof outMotor / sizeof outMotor[0]));
    return m;
}

gaalign::Motor gaalign::Motor::operator+(const Motor &other) const {
    Motor result;
    for(int j=0; j<data.size(); j++) {
        result.data[j] = data[j] + other.data[j];
    }
    return result;
}

gaalign::Motor &gaalign::Motor::operator+=(const gaalign::Motor &other) {
    for(int j=0; j<data.size(); j++) {
        data[j] += other.data[j];
    }

    return *this;
}

Eigen::Matrix4d gaalign::Motor::toTransformationMatrix() const {
    // First transform the origin with the motor to get the translation
    Eigen::Vector3d translation = transformPointWithMotor(Eigen::Vector3d(0, 0, 0), *this);

    // Also transform the x, y and z axis to later form a rotation matrix
    Eigen::Vector3d xAxis = transformPointWithMotor(Eigen::Vector3d(1, 0, 0), *this);
    Eigen::Vector3d yAxis = transformPointWithMotor(Eigen::Vector3d(0, 1, 0), *this);
    Eigen::Vector3d zAxis = transformPointWithMotor(Eigen::Vector3d(0, 0, 1), *this);

    // Remove the translation component from the axes
    xAxis -= translation;
    yAxis -= translation;
    zAxis -= translation;

    // Build a 4x4 matrix from the components
    Eigen::Matrix4d mat;
    mat.block(0,0,3,1) = xAxis;
    mat.block(0,1,3,1) = yAxis;
    mat.block(0,2,3,1) = zAxis;
    mat.block(0,3,3,1) = translation;
    mat(3,3) = 1;


    return mat;
}

gaalign::Motor gaalign::Motor::inverse() const{
    // Get both raw data pointers
    const double* motorArr = data.data();

    // Init output
    double outMotor[8] = {0.0};



#line 186 "D:/Development/GAAlign/src/geometry/motor.cpg"
#include <math.h>
//#pragma gpc multivector normed_reverse
double normed_reverse[8];
//#pragma gpc multivector reverse
double reverse[6];
//#pragma gpc multivector reverse_norm
double reverse_norm;

//#pragma gpc multivector_component reverse e0^e1 reverse[0]
reverse[0] = (-motorArr[1]);
//#pragma gpc multivector_component reverse e0^e2 reverse[1]
reverse[1] = (-motorArr[2]);
//#pragma gpc multivector_component reverse e0^e3 reverse[2]
reverse[2] = (-motorArr[3]);
//#pragma gpc multivector_component reverse e1^e2 reverse[3]
reverse[3] = (-motorArr[4]);
//#pragma gpc multivector_component reverse e1^e3 reverse[4]
reverse[4] = (-motorArr[5]);
//#pragma gpc multivector_component reverse e2^e3 reverse[5]
reverse[5] = (-motorArr[6]);
//#pragma gpc multivector_component reverse_norm 1.0 reverse_norm
reverse_norm = sqrtf(fabs(motorArr[0] * motorArr[0] + (-(reverse[3] * (-reverse[3]))) + (-(reverse[4] * (-reverse[4]))) + (-(reverse[5] * (-reverse[5])))));
//#pragma gpc multivector_component normed_reverse 1.0 normed_reverse[0]
normed_reverse[0] = motorArr[0] * reverse_norm / (reverse_norm * reverse_norm);
//#pragma gpc multivector_component normed_reverse e0^e1 normed_reverse[1]
normed_reverse[1] = reverse[0] * reverse_norm / (reverse_norm * reverse_norm);
//#pragma gpc multivector_component normed_reverse e0^e2 normed_reverse[2]
normed_reverse[2] = reverse[1] * reverse_norm / (reverse_norm * reverse_norm);
//#pragma gpc multivector_component normed_reverse e0^e3 normed_reverse[3]
normed_reverse[3] = reverse[2] * reverse_norm / (reverse_norm * reverse_norm);
//#pragma gpc multivector_component normed_reverse e1^e2 normed_reverse[4]
normed_reverse[4] = reverse[3] * reverse_norm / (reverse_norm * reverse_norm);
//#pragma gpc multivector_component normed_reverse e1^e3 normed_reverse[5]
normed_reverse[5] = reverse[4] * reverse_norm / (reverse_norm * reverse_norm);
//#pragma gpc multivector_component normed_reverse e2^e3 normed_reverse[6]
normed_reverse[6] = reverse[5] * reverse_norm / (reverse_norm * reverse_norm);
//#pragma gpc multivector_component normed_reverse e0^e1^e2^e3 normed_reverse[7]
normed_reverse[7] = motorArr[7] * reverse_norm / (reverse_norm * reverse_norm);

#line 195 "D:/Development/GAAlign/src/geometry/motor.cpg"


outMotor[0] = normed_reverse[0];
outMotor[1] = normed_reverse[1];
outMotor[2] = normed_reverse[2];
outMotor[3] = normed_reverse[3];
outMotor[4] = normed_reverse[4];
outMotor[5] = normed_reverse[5];
outMotor[6] = normed_reverse[6];
outMotor[7] = normed_reverse[7];



#line 200 "D:/Development/GAAlign/src/geometry/motor.cpg"

    // Copy data to Motor struct
    gaalign::Motor m;
    m.data = std::vector<double>(outMotor, outMotor + (sizeof outMotor / sizeof outMotor[0]));
    return m;
}


