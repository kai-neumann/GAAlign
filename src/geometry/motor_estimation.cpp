#line 1 "D:/Development/GAAlign/src/geometry/motor_estimation.cpg"
//
// Created by Kai on 18.01.2022.
//

#include "D:/Development/GAAlign/src/geometry/motor_estimation.h"

void gaalign::estimateMotorFromThreeCorrespondences(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& correspondences, Motor& outputMotor) {
    // Check if there are exactly three correspondences
    if(correspondences.size() != 3) {
        std::cerr << "Unable to estimate motor from a different number of correspondences than 3!" << std::endl;
        throw std::runtime_error("Invalid arguments for 'estimateMotorFromThreeCorrespondences'. Requires exactly three correspondences!");
    }

    double calculatedMotor[8] = {0.0};

    // Convert the points into arrays that can be accessed
    double A_src_arr[4] = {correspondences[0].first.x(), correspondences[0].first.y(), correspondences[0].first.z(), 1.0};
    double B_src_arr[4] = {correspondences[1].first.x(), correspondences[1].first.y(), correspondences[1].first.z(), 1.0};
    double C_src_arr[4] = {correspondences[2].first.x(), correspondences[2].first.y(), correspondences[2].first.z(), 1.0};
    double A_tar_arr[4] = {correspondences[0].second.x(), correspondences[0].second.y(), correspondences[0].second.z(), 1.0};
    double B_tar_arr[4] = {correspondences[1].second.x(), correspondences[1].second.y(), correspondences[1].second.z(), 1.0};
    double C_tar_arr[4] = {correspondences[2].second.x(), correspondences[2].second.y(), correspondences[2].second.z(), 1.0};



#line 33 "D:/Development/GAAlign/src/geometry/motor_estimation.cpg"
#include <math.h>
//#pragma gpc multivector A_src
double A_src[2];
//#pragma gpc multivector A_tar
double A_tar[2];
//#pragma gpc multivector B2
double B2[4];
//#pragma gpc multivector B_src
double B_src[2];
//#pragma gpc multivector B_tar
double B_tar[2];
//#pragma gpc multivector C2
double C2[4];
//#pragma gpc multivector C3
double C3[4];
//#pragma gpc multivector C_src
double C_src[2];
//#pragma gpc multivector C_tar
double C_tar[2];
//#pragma gpc multivector combined_motor
double combined_motor[8];
//#pragma gpc multivector L1
double L1[6];
//#pragma gpc multivector L2
double L2[6];
//#pragma gpc multivector motor_norm
double motor_norm;
//#pragma gpc multivector out_motor
double out_motor[8];
//#pragma gpc multivector P1
double P1[4];
//#pragma gpc multivector P2
double P2[4];
//#pragma gpc multivector VA
double VA[4];
//#pragma gpc multivector VA_norm
double VA_norm;
//#pragma gpc multivector VA_unnormalized
double VA_unnormalized[4];
//#pragma gpc multivector VB
double VB[8];
//#pragma gpc multivector VB_norm
double VB_norm;
//#pragma gpc multivector VB_unnormalized
double VB_unnormalized[8];
//#pragma gpc multivector VC
double VC[7];
//#pragma gpc multivector VC_norm
double VC_norm;
//#pragma gpc multivector VC_unnormalized
double VC_unnormalized[7];

//#pragma gpc multivector_component A_src e0^e1^e2 A_src[0]
A_src[0] = (-A_src_arr[2]);
//#pragma gpc multivector_component A_src e0^e2^e3 A_src[1]
A_src[1] = (-A_src_arr[0]);
//#pragma gpc multivector_component B_src e0^e1^e2 B_src[0]
B_src[0] = (-B_src_arr[2]);
//#pragma gpc multivector_component B_src e0^e2^e3 B_src[1]
B_src[1] = (-B_src_arr[0]);
//#pragma gpc multivector_component C_src e0^e1^e2 C_src[0]
C_src[0] = (-C_src_arr[2]);
//#pragma gpc multivector_component C_src e0^e2^e3 C_src[1]
C_src[1] = (-C_src_arr[0]);
//#pragma gpc multivector_component A_tar e0^e1^e2 A_tar[0]
A_tar[0] = (-A_tar_arr[2]);
//#pragma gpc multivector_component A_tar e0^e2^e3 A_tar[1]
A_tar[1] = (-A_tar_arr[0]);
//#pragma gpc multivector_component B_tar e0^e1^e2 B_tar[0]
B_tar[0] = (-B_tar_arr[2]);
//#pragma gpc multivector_component B_tar e0^e2^e3 B_tar[1]
B_tar[1] = (-B_tar_arr[0]);
//#pragma gpc multivector_component C_tar e0^e1^e2 C_tar[0]
C_tar[0] = (-C_tar_arr[2]);
//#pragma gpc multivector_component C_tar e0^e2^e3 C_tar[1]
C_tar[1] = (-C_tar_arr[0]);
//#pragma gpc multivector_component VA_unnormalized 1.0 VA_unnormalized[0]
VA_unnormalized[0] = 1.0 + (-(A_tar_arr[3] * (-A_src_arr[3]) / (-(A_src_arr[3] * (-A_src_arr[3])))));
//#pragma gpc multivector_component VA_unnormalized e0^e1 VA_unnormalized[1]
VA_unnormalized[1] = (-(A_tar[1] * (-A_src_arr[3]) / (-(A_src_arr[3] * (-A_src_arr[3]))))) + A_tar_arr[3] * (-A_src[1]) / (-(A_src_arr[3] * (-A_src_arr[3])));
//#pragma gpc multivector_component VA_unnormalized e0^e2 VA_unnormalized[2]
VA_unnormalized[2] = A_tar_arr[1] * (-A_src_arr[3]) / (-(A_src_arr[3] * (-A_src_arr[3]))) + (-(A_tar_arr[3] * (-A_src_arr[1]) / (-(A_src_arr[3] * (-A_src_arr[3])))));
//#pragma gpc multivector_component VA_unnormalized e0^e3 VA_unnormalized[3]
VA_unnormalized[3] = (-(A_tar[0] * (-A_src_arr[3]) / (-(A_src_arr[3] * (-A_src_arr[3]))))) + A_tar_arr[3] * (-A_src[0]) / (-(A_src_arr[3] * (-A_src_arr[3])));
//#pragma gpc multivector_component VA_norm 1.0 VA_norm
VA_norm = sqrtf(fabs(VA_unnormalized[0] * VA_unnormalized[0]));
//#pragma gpc multivector_component VA 1.0 VA[0]
VA[0] = VA_unnormalized[0] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component VA e0^e1 VA[1]
VA[1] = VA_unnormalized[1] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component VA e0^e2 VA[2]
VA[2] = VA_unnormalized[2] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component VA e0^e3 VA[3]
VA[3] = VA_unnormalized[3] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component B2 e0^e1^e2 B2[0]
B2[0] = (VA[0] * B_src[0] + VA[3] * B_src_arr[3]) * VA[0] + (-(VA[0] * B_src_arr[3] * (-VA[3])));
//#pragma gpc multivector_component B2 e0^e1^e3 B2[1]
B2[1] = (VA[0] * B_src_arr[1] + (-(VA[2] * B_src_arr[3]))) * VA[0] + VA[0] * B_src_arr[3] * (-VA[2]);
//#pragma gpc multivector_component B2 e0^e2^e3 B2[2]
B2[2] = (VA[0] * B_src[1] + VA[1] * B_src_arr[3]) * VA[0] + (-(VA[0] * B_src_arr[3] * (-VA[1])));
//#pragma gpc multivector_component B2 e1^e2^e3 B2[3]
B2[3] = VA[0] * B_src_arr[3] * VA[0];
//#pragma gpc multivector_component C2 e0^e1^e2 C2[0]
C2[0] = (VA[0] * C_src[0] + VA[3] * C_src_arr[3]) * VA[0] + (-(VA[0] * C_src_arr[3] * (-VA[3])));
//#pragma gpc multivector_component C2 e0^e1^e3 C2[1]
C2[1] = (VA[0] * C_src_arr[1] + (-(VA[2] * C_src_arr[3]))) * VA[0] + VA[0] * C_src_arr[3] * (-VA[2]);
//#pragma gpc multivector_component C2 e0^e2^e3 C2[2]
C2[2] = (VA[0] * C_src[1] + VA[1] * C_src_arr[3]) * VA[0] + (-(VA[0] * C_src_arr[3] * (-VA[1])));
//#pragma gpc multivector_component C2 e1^e2^e3 C2[3]
C2[3] = VA[0] * C_src_arr[3] * VA[0];
//#pragma gpc multivector_component L1 e0^e1 L1[0]
L1[0] = A_tar_arr[1] * (-B_tar[0]) + (-((-A_tar[0]) * B_tar_arr[1]));
//#pragma gpc multivector_component L1 e0^e2 L1[1]
L1[1] = (-((-A_tar[1]) * (-B_tar[0]) + (-((-A_tar[0]) * (-B_tar[1])))));
//#pragma gpc multivector_component L1 e0^e3 L1[2]
L1[2] = (-A_tar[1]) * B_tar_arr[1] + (-(A_tar_arr[1] * (-B_tar[1])));
//#pragma gpc multivector_component L1 e1^e2 L1[3]
L1[3] = A_tar_arr[3] * (-B_tar[0]) + (-((-A_tar[0]) * B_tar_arr[3]));
//#pragma gpc multivector_component L1 e1^e3 L1[4]
L1[4] = (-(A_tar_arr[3] * B_tar_arr[1] + (-(A_tar_arr[1] * B_tar_arr[3]))));
//#pragma gpc multivector_component L1 e2^e3 L1[5]
L1[5] = A_tar_arr[3] * (-B_tar[1]) + (-((-A_tar[1]) * B_tar_arr[3]));
//#pragma gpc multivector_component L2 e0^e1 L2[0]
L2[0] = A_tar_arr[1] * (-B2[0]) + (-((-A_tar[0]) * B2[1]));
//#pragma gpc multivector_component L2 e0^e2 L2[1]
L2[1] = (-((-A_tar[1]) * (-B2[0]) + (-((-A_tar[0]) * (-B2[2])))));
//#pragma gpc multivector_component L2 e0^e3 L2[2]
L2[2] = (-A_tar[1]) * B2[1] + (-(A_tar_arr[1] * (-B2[2])));
//#pragma gpc multivector_component L2 e1^e2 L2[3]
L2[3] = A_tar_arr[3] * (-B2[0]) + (-((-A_tar[0]) * B2[3]));
//#pragma gpc multivector_component L2 e1^e3 L2[4]
L2[4] = (-(A_tar_arr[3] * B2[1] + (-(A_tar_arr[1] * B2[3]))));
//#pragma gpc multivector_component L2 e2^e3 L2[5]
L2[5] = A_tar_arr[3] * (-B2[2]) + (-((-A_tar[1]) * B2[3]));
//#pragma gpc multivector_component VB_unnormalized 1.0 VB_unnormalized[0]
VB_unnormalized[0] = 1.0 + (-(L1[3] * (-L2[3]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))))) + (-(L1[4] * (-L2[4]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))))) + (-(L1[5] * (-L2[5]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))))));
//#pragma gpc multivector_component VB_unnormalized e0^e1 VB_unnormalized[1]
VB_unnormalized[1] = (-(L1[1] * (-L2[3]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))))) + (-(L1[2] * (-L2[4]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))))) + L1[3] * (-L2[1]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))) + L1[4] * (-L2[2]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))));
//#pragma gpc multivector_component VB_unnormalized e0^e2 VB_unnormalized[2]
VB_unnormalized[2] = L1[0] * (-L2[3]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))) + (-(L1[2] * (-L2[5]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))))) + (-(L1[3] * (-L2[0]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))))) + L1[5] * (-L2[2]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))));
//#pragma gpc multivector_component VB_unnormalized e0^e3 VB_unnormalized[3]
VB_unnormalized[3] = L1[0] * (-L2[4]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))) + L1[1] * (-L2[5]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))) + (-(L1[4] * (-L2[0]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))))) + (-(L1[5] * (-L2[1]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))))));
//#pragma gpc multivector_component VB_unnormalized e1^e2 VB_unnormalized[4]
VB_unnormalized[4] = (-(L1[4] * (-L2[5]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))))) + L1[5] * (-L2[4]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))));
//#pragma gpc multivector_component VB_unnormalized e1^e3 VB_unnormalized[5]
VB_unnormalized[5] = L1[3] * (-L2[5]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))) + (-(L1[5] * (-L2[3]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))))));
//#pragma gpc multivector_component VB_unnormalized e2^e3 VB_unnormalized[6]
VB_unnormalized[6] = (-(L1[3] * (-L2[4]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))))) + L1[4] * (-L2[3]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))));
//#pragma gpc multivector_component VB_unnormalized e0^e1^e2^e3 VB_unnormalized[7]
VB_unnormalized[7] = L1[0] * (-L2[5]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))) + (-(L1[1] * (-L2[4]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))))) + L1[2] * (-L2[3]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))) + L1[3] * (-L2[2]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))) + (-(L1[4] * (-L2[1]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5])))))) + L1[5] * (-L2[0]) / ((-(L2[3] * (-L2[3]))) + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))));
//#pragma gpc multivector_component VB_norm 1.0 VB_norm
VB_norm = sqrtf(fabs(VB_unnormalized[0] * VB_unnormalized[0] + (-(VB_unnormalized[4] * (-VB_unnormalized[4]))) + (-(VB_unnormalized[5] * (-VB_unnormalized[5]))) + (-(VB_unnormalized[6] * (-VB_unnormalized[6])))));
//#pragma gpc multivector_component VB 1.0 VB[0]
VB[0] = VB_unnormalized[0] * VB_norm / (VB_norm * VB_norm);
//#pragma gpc multivector_component VB e0^e1 VB[1]
VB[1] = VB_unnormalized[1] * VB_norm / (VB_norm * VB_norm);
//#pragma gpc multivector_component VB e0^e2 VB[2]
VB[2] = VB_unnormalized[2] * VB_norm / (VB_norm * VB_norm);
//#pragma gpc multivector_component VB e0^e3 VB[3]
VB[3] = VB_unnormalized[3] * VB_norm / (VB_norm * VB_norm);
//#pragma gpc multivector_component VB e1^e2 VB[4]
VB[4] = VB_unnormalized[4] * VB_norm / (VB_norm * VB_norm);
//#pragma gpc multivector_component VB e1^e3 VB[5]
VB[5] = VB_unnormalized[5] * VB_norm / (VB_norm * VB_norm);
//#pragma gpc multivector_component VB e2^e3 VB[6]
VB[6] = VB_unnormalized[6] * VB_norm / (VB_norm * VB_norm);
//#pragma gpc multivector_component VB e0^e1^e2^e3 VB[7]
VB[7] = VB_unnormalized[7] * VB_norm / (VB_norm * VB_norm);
//#pragma gpc multivector_component C3 e0^e1^e2 C3[0]
C3[0] = ((-(VB[4] * C2[0])) + (-(VB[5] * C2[1])) + (-(VB[6] * C2[2])) + (-(VB[7] * C2[3]))) * (-VB[4]) + (-((-(VB[6] * C2[3])) * (-VB[2]))) + VB[5] * C2[3] * (-VB[1]) + (-((-(VB[4] * C2[3])) * VB[7])) + (VB[0] * C2[0] + VB[3] * C2[3] + (-(VB[5] * C2[2])) + VB[6] * C2[1]) * VB[0] + (-((VB[0] * C2[1] + (-(VB[2] * C2[3])) + VB[4] * C2[2] + (-(VB[6] * C2[0]))) * (-VB[6]))) + (VB[0] * C2[2] + VB[1] * C2[3] + (-(VB[4] * C2[1])) + VB[5] * C2[0]) * (-VB[5]) + (-(VB[0] * C2[3] * (-VB[3])));
//#pragma gpc multivector_component C3 e0^e1^e3 C3[1]
C3[1] = ((-(VB[4] * C2[0])) + (-(VB[5] * C2[1])) + (-(VB[6] * C2[2])) + (-(VB[7] * C2[3]))) * (-VB[5]) + (-((-(VB[6] * C2[3])) * (-VB[3]))) + VB[5] * C2[3] * VB[7] + (-(VB[4] * C2[3])) * (-VB[1]) + (VB[0] * C2[0] + VB[3] * C2[3] + (-(VB[5] * C2[2])) + VB[6] * C2[1]) * (-VB[6]) + (VB[0] * C2[1] + (-(VB[2] * C2[3])) + VB[4] * C2[2] + (-(VB[6] * C2[0]))) * VB[0] + (-((VB[0] * C2[2] + VB[1] * C2[3] + (-(VB[4] * C2[1])) + VB[5] * C2[0]) * (-VB[4]))) + VB[0] * C2[3] * (-VB[2]);
//#pragma gpc multivector_component C3 e0^e2^e3 C3[2]
C3[2] = ((-(VB[4] * C2[0])) + (-(VB[5] * C2[1])) + (-(VB[6] * C2[2])) + (-(VB[7] * C2[3]))) * (-VB[6]) + (-((-(VB[6] * C2[3])) * VB[7])) + (-(VB[5] * C2[3] * (-VB[3]))) + (-(VB[4] * C2[3])) * (-VB[2]) + (-((VB[0] * C2[0] + VB[3] * C2[3] + (-(VB[5] * C2[2])) + VB[6] * C2[1]) * (-VB[5]))) + (VB[0] * C2[1] + (-(VB[2] * C2[3])) + VB[4] * C2[2] + (-(VB[6] * C2[0]))) * (-VB[4]) + (VB[0] * C2[2] + VB[1] * C2[3] + (-(VB[4] * C2[1])) + VB[5] * C2[0]) * VB[0] + (-(VB[0] * C2[3] * (-VB[1])));
//#pragma gpc multivector_component C3 e1^e2^e3 C3[3]
C3[3] = (-(VB[6] * C2[3])) * (-VB[6]) + (-(VB[5] * C2[3] * (-VB[5]))) + (-(VB[4] * C2[3])) * (-VB[4]) + VB[0] * C2[3] * VB[0];
//#pragma gpc multivector_component P1 e0 P1[0]
P1[0] = L1[2] * (-C_tar[0]) + (-((-L1[1]) * C_tar_arr[1])) + L1[0] * (-C_tar[1]);
//#pragma gpc multivector_component P1 e1 P1[1]
P1[1] = (-((-L1[4]) * (-C_tar[0]) + (-(L1[3] * C_tar_arr[1])) + L1[0] * C_tar_arr[3]));
//#pragma gpc multivector_component P1 e2 P1[2]
P1[2] = L1[5] * (-C_tar[0]) + (-(L1[3] * (-C_tar[1]))) + (-L1[1]) * C_tar_arr[3];
//#pragma gpc multivector_component P1 e3 P1[3]
P1[3] = (-(L1[5] * C_tar_arr[1] + (-((-L1[4]) * (-C_tar[1]))) + L1[2] * C_tar_arr[3]));
//#pragma gpc multivector_component P2 e0 P2[0]
P2[0] = L1[2] * (-C3[0]) + (-((-L1[1]) * C3[1])) + L1[0] * (-C3[2]);
//#pragma gpc multivector_component P2 e1 P2[1]
P2[1] = (-((-L1[4]) * (-C3[0]) + (-(L1[3] * C3[1])) + L1[0] * C3[3]));
//#pragma gpc multivector_component P2 e2 P2[2]
P2[2] = L1[5] * (-C3[0]) + (-(L1[3] * (-C3[2]))) + (-L1[1]) * C3[3];
//#pragma gpc multivector_component P2 e3 P2[3]
P2[3] = (-(L1[5] * C3[1] + (-((-L1[4]) * (-C3[2]))) + L1[2] * C3[3]));
//#pragma gpc multivector_component VC_unnormalized 1.0 VC_unnormalized[0]
VC_unnormalized[0] = 1.0 + P1[1] * P2[1] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3]) + P1[2] * P2[2] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3]) + P1[3] * P2[3] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3]);
//#pragma gpc multivector_component VC_unnormalized e0^e1 VC_unnormalized[1]
VC_unnormalized[1] = P1[0] * P2[1] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3]) + (-(P1[1] * P2[0] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3])));
//#pragma gpc multivector_component VC_unnormalized e0^e2 VC_unnormalized[2]
VC_unnormalized[2] = P1[0] * P2[2] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3]) + (-(P1[2] * P2[0] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3])));
//#pragma gpc multivector_component VC_unnormalized e0^e3 VC_unnormalized[3]
VC_unnormalized[3] = P1[0] * P2[3] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3]) + (-(P1[3] * P2[0] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3])));
//#pragma gpc multivector_component VC_unnormalized e1^e2 VC_unnormalized[4]
VC_unnormalized[4] = P1[1] * P2[2] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3]) + (-(P1[2] * P2[1] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3])));
//#pragma gpc multivector_component VC_unnormalized e1^e3 VC_unnormalized[5]
VC_unnormalized[5] = P1[1] * P2[3] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3]) + (-(P1[3] * P2[1] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3])));
//#pragma gpc multivector_component VC_unnormalized e2^e3 VC_unnormalized[6]
VC_unnormalized[6] = P1[2] * P2[3] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3]) + (-(P1[3] * P2[2] / (P2[1] * P2[1] + P2[2] * P2[2] + P2[3] * P2[3])));
//#pragma gpc multivector_component VC_norm 1.0 VC_norm
VC_norm = sqrtf(fabs(VC_unnormalized[0] * VC_unnormalized[0] + (-(VC_unnormalized[4] * (-VC_unnormalized[4]))) + (-(VC_unnormalized[5] * (-VC_unnormalized[5]))) + (-(VC_unnormalized[6] * (-VC_unnormalized[6])))));
//#pragma gpc multivector_component VC 1.0 VC[0]
VC[0] = VC_unnormalized[0] * VC_norm / (VC_norm * VC_norm);
//#pragma gpc multivector_component VC e0^e1 VC[1]
VC[1] = VC_unnormalized[1] * VC_norm / (VC_norm * VC_norm);
//#pragma gpc multivector_component VC e0^e2 VC[2]
VC[2] = VC_unnormalized[2] * VC_norm / (VC_norm * VC_norm);
//#pragma gpc multivector_component VC e0^e3 VC[3]
VC[3] = VC_unnormalized[3] * VC_norm / (VC_norm * VC_norm);
//#pragma gpc multivector_component VC e1^e2 VC[4]
VC[4] = VC_unnormalized[4] * VC_norm / (VC_norm * VC_norm);
//#pragma gpc multivector_component VC e1^e3 VC[5]
VC[5] = VC_unnormalized[5] * VC_norm / (VC_norm * VC_norm);
//#pragma gpc multivector_component VC e2^e3 VC[6]
VC[6] = VC_unnormalized[6] * VC_norm / (VC_norm * VC_norm);
//#pragma gpc multivector_component combined_motor 1.0 combined_motor[0]
combined_motor[0] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[0];
//#pragma gpc multivector_component combined_motor e0^e1 combined_motor[1]
combined_motor[1] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[1] + (VC[0] * VB[1] + VC[1] * VB[0] + (-(VC[2] * VB[4])) + (-(VC[3] * VB[5])) + VC[4] * VB[2] + VC[5] * VB[3] + (-(VC[6] * VB[7]))) * VA[0] + (VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[2] + (VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[3];
//#pragma gpc multivector_component combined_motor e0^e2 combined_motor[2]
combined_motor[2] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[2] + (VC[0] * VB[2] + VC[1] * VB[4] + VC[2] * VB[0] + (-(VC[3] * VB[6])) + (-(VC[4] * VB[1])) + VC[5] * VB[7] + VC[6] * VB[3]) * VA[0] + (-((VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[1])) + (VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[3];
//#pragma gpc multivector_component combined_motor e0^e3 combined_motor[3]
combined_motor[3] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[3] + (VC[0] * VB[3] + VC[1] * VB[5] + VC[2] * VB[6] + VC[3] * VB[0] + (-(VC[4] * VB[7])) + (-(VC[5] * VB[1])) + (-(VC[6] * VB[2]))) * VA[0] + (-((VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[1])) + (-((VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[2]));
//#pragma gpc multivector_component combined_motor e1^e2 combined_motor[4]
combined_motor[4] = (VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[0];
//#pragma gpc multivector_component combined_motor e1^e3 combined_motor[5]
combined_motor[5] = (VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[0];
//#pragma gpc multivector_component combined_motor e2^e3 combined_motor[6]
combined_motor[6] = (VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[0];
//#pragma gpc multivector_component combined_motor e0^e1^e2^e3 combined_motor[7]
combined_motor[7] = (VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[3] + (-((VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[2])) + (VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[1] + (VC[0] * VB[7] + VC[1] * VB[6] + (-(VC[2] * VB[5])) + VC[3] * VB[4] + VC[4] * VB[3] + (-(VC[5] * VB[2])) + VC[6] * VB[1]) * VA[0];
//#pragma gpc multivector_component motor_norm 1.0 motor_norm
motor_norm = sqrtf(fabs(combined_motor[0] * combined_motor[0] + (-(combined_motor[4] * (-combined_motor[4]))) + (-(combined_motor[5] * (-combined_motor[5]))) + (-(combined_motor[6] * (-combined_motor[6])))));
//#pragma gpc multivector_component out_motor 1.0 out_motor[0]
out_motor[0] = combined_motor[0] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component out_motor e0^e1 out_motor[1]
out_motor[1] = combined_motor[1] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component out_motor e0^e2 out_motor[2]
out_motor[2] = combined_motor[2] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component out_motor e0^e3 out_motor[3]
out_motor[3] = combined_motor[3] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component out_motor e1^e2 out_motor[4]
out_motor[4] = combined_motor[4] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component out_motor e1^e3 out_motor[5]
out_motor[5] = combined_motor[5] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component out_motor e2^e3 out_motor[6]
out_motor[6] = combined_motor[6] * motor_norm / (motor_norm * motor_norm);
//#pragma gpc multivector_component out_motor e0^e1^e2^e3 out_motor[7]
out_motor[7] = combined_motor[7] * motor_norm / (motor_norm * motor_norm);

#line 84 "D:/Development/GAAlign/src/geometry/motor_estimation.cpg"


calculatedMotor[0] = out_motor[0];
calculatedMotor[1] = out_motor[1];
calculatedMotor[2] = out_motor[2];
calculatedMotor[3] = out_motor[3];
calculatedMotor[4] = out_motor[4];
calculatedMotor[5] = out_motor[5];
calculatedMotor[6] = out_motor[6];
calculatedMotor[7] = out_motor[7];



#line 89 "D:/Development/GAAlign/src/geometry/motor_estimation.cpg"


    // Set to output
    for(int i=0; i<8; i++) {
        outputMotor.data[i] = calculatedMotor[i];
    }
}

void gaalign::sampleSubIndices(int vectorSize, int groupSize, std::vector<int>& outArray) {
    // Iterate until the correct size was achieved
    while (outArray.size() < groupSize) {
        // Get a random index
        int index = rand() % vectorSize;

        // If it does not exist already: Add it
        if (std::find(outArray.begin(), outArray.end(), index) == outArray.end()) {
            outArray.push_back(index);
        }
    }
}
