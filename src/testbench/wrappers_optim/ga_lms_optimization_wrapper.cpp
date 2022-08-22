//
// Created by Kai on 03.05.2022.
//

#include "ga_lms_optimization_wrapper.h"

/*
 * This class is largely based on https://github.com/wilderlopes/OpenGA/blob/master/scripts/GAAFs_poseEstimation/Simulations/Scripts/Cpp/GA-LMS_bunny/main.cpp
 */

#define _USE_MATH_DEFINES
#include <boost/thread/thread.hpp>
#include <cmath>
#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#include <iterator> // for ostream_iterator
#include <math.h>       /* log10 */
#include <stdio.h>
#include <fstream>
#include <string>

#include "gaalet/gaalet.h"
#include <algorithm>
#include <chrono>
#include <random>

using namespace std;
using namespace Eigen;

std::string gaalign::GA_LMS_OptimizationWrapper::getName() const {
    if(m_useSteepestDecent) {
        return "GA-LMS (SD)";
    }
    return "GA-LMS";
}

int myrandom (int i) { return std::rand()%i;}

std::pair<Eigen::Matrix4d, double>
gaalign::GA_LMS_OptimizationWrapper::calculateRegistration(const std::vector<Correspondence> &correspondences,
                                                           const std::vector<Correspondence> &correspondingNormals) const {

    // Source = X
    // Target = Y


    //Gnuplot gp;

    double error_galms, cost_function_mag;
    double cost_function = 0;
    //double mu_galms = 0.1;
    //double mu_steep = mu_galms/4;
    double mu_galms = 0.1;
    double mu_steep = 0.1;
    double K;
    int iter_value;

    std::vector<double> CF_galms;
    std::vector<double> MSE_steep;
    std::vector<double> MSE_galms;
    std::vector<int> myvector;

    typedef gaalet::algebra<gaalet::signature<3,0> > em;

    bool limitIterations = false;
    int maxIterations = 100;

    //=======================================================================================
    /*
    Code for scrambling the correspondences each time the binary is called by Matlab. Only
    the loops related to the cleaned correspondences are being scrambled. The ones related
    to the good correspondences stay untouched.
    */
    for (size_t i=0; i < correspondences.size(); i++) myvector.push_back((int)i);
    srand((std::random_device())());
    std::mt19937 re((std::random_device())());
    std::shuffle(myvector.begin(), myvector.end(), re);

    // cut to max iterations
    if(limitIterations) {
        myvector.resize(100);
    }

    // CENTROIDS ============================================================================
    em::mv<1, 2, 4>::type y_sum{0, 0, 0};
    em::mv<1, 2, 4>::type x_sum{0, 0, 0};
    em::mv<1, 2, 4>::type y_cent{0, 0, 0};
    em::mv<1, 2, 4>::type x_cent{0, 0, 0};

    // calculating centroids
    for (std::vector<int>::iterator it=myvector.begin(); it!=myvector.end(); ++it)
    {
        iter_value = *it;

        y_sum[0] += correspondences[iter_value].second.x();
        y_sum[1] += correspondences[iter_value].second.y();
        y_sum[2] += correspondences[iter_value].second.z();
        //y_sum += Y_PCDPtr->points[i];
    }

    y_cent[0] = y_sum[0]/((double)correspondences.size());
    y_cent[1] = y_sum[1]/((double)correspondences.size());
    y_cent[2] = y_sum[2]/((double)correspondences.size());

    for (std::vector<int>::iterator it=myvector.begin(); it!=myvector.end(); ++it)
    {
        iter_value = *it;

        x_sum[0] += correspondences[iter_value].first.x();
        x_sum[1] += correspondences[iter_value].first.y();
        x_sum[2] += correspondences[iter_value].first.z();
        //x_sum += X_PCDPtr->points[i];
    }

    x_cent[0] = x_sum[0]/((double)correspondences.size());
    x_cent[1] = x_sum[1]/((double)correspondences.size());
    x_cent[2] = x_sum[2]/((double)correspondences.size());


    // Generating the new point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr Y_PCDPtr_cent (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr X_PCDPtr_cent (new pcl::PointCloud<pcl::PointXYZ>);

    for (std::vector<int>::iterator it=myvector.begin(); it!=myvector.end(); ++it)
    {
        iter_value = *it;

        pcl::PointXYZ basic_point_3;
        basic_point_3.x =  correspondences[iter_value].second.x() - y_cent[0];
        basic_point_3.y =  correspondences[iter_value].second.y() - y_cent[1];
        basic_point_3.z =  correspondences[iter_value].second.z() - y_cent[2];
        Y_PCDPtr_cent->points.push_back(basic_point_3);

    }

    for (std::vector<int>::iterator it=myvector.begin(); it!=myvector.end(); ++it)
    {
        iter_value = *it;

        pcl::PointXYZ basic_point_4;
        basic_point_4.x = correspondences[iter_value].first.x() - x_cent[0];
        basic_point_4.y = correspondences[iter_value].first.y() - x_cent[1];
        basic_point_4.z = correspondences[iter_value].first.z() - x_cent[2];
        X_PCDPtr_cent->points.push_back(basic_point_4);
    }

    // Initializing rotors in gaalet
    em::mv<0, 3, 5, 6>::type r_old{0.5, 0.5, -0.5, 0.5};
    em::mv<0, 3, 5, 6>::type r_new{0, 0, 0, 0};
    em::mv<0, 3, 5, 6>::type r_steep_old{0.5, 0.5, -0.5, 0.5};
    em::mv<0, 3, 5, 6>::type r_steep_new{0, 0, 0, 0};

    // Initializing auxiliary points in gaalet
    em::mv<1, 2, 4>::type y{0, 0, 0};
    em::mv<1, 2, 4>::type x{0, 0, 0};
    em::mv<1, 2, 4>::type x_reg{0, 0, 0};
    em::mv<1, 2, 4>::type y_CF{0, 0, 0};
    em::mv<1, 2, 4>::type x_CF{0, 0, 0};
    em::mv<1, 2, 4>::type y_steep{0, 0, 0};
    em::mv<1, 2, 4>::type x_steep{0, 0, 0};

    em::mv<0,1,2,3,4,5,6,7>::type error_steep{0, 0, 0, 0, 0, 0, 0, 0};
    em::mv<0,1,2,3,4,5,6,7>::type zero_multivector{0, 0, 0, 0, 0, 0, 0, 0};

    K = (double)correspondences.size(); //Number of points in the PCDs
    if(limitIterations) {
        K = maxIterations;
    }

    // Start the time measurement
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    int iterationCount = 0;

    //Begining of adaptation loop
    for (std::vector<int>::iterator it=myvector.begin(); it!=myvector.end(); ++it)
    {
        // Extract the iterator value
        iter_value = *it;

        // Increase iteration count
        iterationCount++;
        if(limitIterations && iterationCount > maxIterations) break;

        y[0] = Y_PCDPtr_cent->points[iter_value].x;
        y[1] = Y_PCDPtr_cent->points[iter_value].y;
        y[2] = Y_PCDPtr_cent->points[iter_value].z;

        x[0] = X_PCDPtr_cent->points[iter_value].x;
        x[1] = X_PCDPtr_cent->points[iter_value].y;
        x[2] = X_PCDPtr_cent->points[iter_value].z;

        if(m_useSteepestDecent) {
            //Steepest-Descent ===============================================
            //Calculating the error for the steepest-descent algorithm (when the AF uses all the vailable K correspondence pairs)
            for (std::vector<int>::iterator it = myvector.begin(); it != myvector.end(); ++it) {
                iter_value = *it;

                y_steep[0] = Y_PCDPtr_cent->points[iter_value].x;
                y_steep[1] = Y_PCDPtr_cent->points[iter_value].y;
                y_steep[2] = Y_PCDPtr_cent->points[iter_value].z;

                x_steep[0] = X_PCDPtr_cent->points[iter_value].x;
                x_steep[1] = X_PCDPtr_cent->points[iter_value].y;
                x_steep[2] = X_PCDPtr_cent->points[iter_value].z;

                error_steep = error_steep + (y_steep ^ (r_steep_old * x_steep *
                                                        (~r_steep_old))); //error to be used in the steepest-descent update rule
                //error_steep += pow(error_steep_mag,2);
            }
            //std::cout << "error_steep" << error_steep << std::endl;


            r_steep_new = r_steep_old + mu_steep*(4/K)*error_steep*r_steep_old;
            r_steep_new = r_steep_new*(1/eval(magnitude(r_steep_new)));
            r_steep_new = eval(r_steep_new);

            r_steep_old = r_steep_new;

            error_steep = zero_multivector;

        }
        else {
            //GA-LMS ===============================================
            //error_galms = (double) eval(magnitude(y - r_old*x*(~r_old)));
            //MSE_galms.push_back(error_galms); // .push_back shifts the previous content of the vector

            r_new = r_old + mu_galms*(y^(r_old*x*(~r_old)))*r_old;
            r_new = r_new*(1/eval(magnitude(r_new)));
            r_new = eval(r_new);

            r_old = r_new;
        }
    }

    // Stop the time measurement
    auto end = std::chrono::high_resolution_clock::now();
    double timeMS = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/1000000.0;

    em::mv<0, 3, 5, 6>::type resultingMotor{0, 0, 0, 0};
    if(m_useSteepestDecent) {
        resultingMotor = r_steep_new;
    }
    else {
        resultingMotor = r_new;
    }

    //std::cout << resultingMotor << std::endl;

    // Extract the translation
    em::mv<1, 2, 4>::type translation{0, 0, 0};
    translation = (resultingMotor*x_cent*(~resultingMotor) - y_cent);

    // Extract the rotation
    em::mv<1, 2, 4>::type xAxis{1, 0, 0};
    em::mv<1, 2, 4>::type yAxis{0, 1, 0};
    em::mv<1, 2, 4>::type zAxis{0, 0, 1};
    xAxis = resultingMotor*xAxis*(~resultingMotor);
    yAxis = resultingMotor*yAxis*(~resultingMotor);
    zAxis = resultingMotor*zAxis*(~resultingMotor);

    // Convert to matrix
    Eigen::Matrix4d mat;
    // x Axis
    mat(0, 0) = xAxis[0];
    mat(1, 0) = xAxis[1];
    mat(2, 0) = xAxis[2];

    // y axis
    mat(0, 1) = yAxis[0];
    mat(1, 1) = yAxis[1];
    mat(2, 1) = yAxis[2];

    // z axis
    mat(0, 2) = zAxis[0];
    mat(1, 2) = zAxis[1];
    mat(2, 2) = zAxis[2];

    // translation
    mat(0, 3) = -translation[0];
    mat(1, 3) = -translation[1];
    mat(2, 3) = -translation[2];

    // Homogenous part
    mat(3, 0) = 0;
    mat(3, 1) = 0;
    mat(3, 2) = 0;
    mat(3, 3) = 1;

    //std::cout << mat << std::endl;

    return std::make_pair(mat, timeMS);
}

gaalign::GA_LMS_OptimizationWrapper::GA_LMS_OptimizationWrapper(bool steepestDescent) {
    m_useSteepestDecent = steepestDescent;
}
