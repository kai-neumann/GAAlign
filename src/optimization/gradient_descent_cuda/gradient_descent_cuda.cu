#line 1 "D:/Development/GAAlign/src/optimization/gradient_descent_cuda/gradient_descent_cuda.cug"
//
// Created by Kai on 23.0f2.2022.
//

#include "D:/Development/GAAlign/src/optimization/gradient_descent_cuda/gradient_descent_cuda.h"

#include <random>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <numeric>

#include <geometry/motor_estimation.h>
#include <optimization/fast_shuffle.h>

// CUDA
#include <cmath>
#include <Eigen/Core>

/*
 * CUDA HELPER FUNCTIONS ===============================================================================================
 */
inline unsigned int div_up(unsigned int numerator, unsigned int denominator)
{
    unsigned int result = numerator / denominator;
    if (numerator % denominator) ++result;
    return result;
}

dim3 getBlockDim() {
    // Choose a fixed (sensible) block size
    int block_size = 256;

    dim3 dimBlock(block_size, 1);
    return dimBlock;
}

dim3 getGridDim(int width, int height, dim3 blockSize) {
    dim3 dimGrid(div_up(width, blockSize.x), div_up(height, blockSize.y));

    return dimGrid;
}

#define CUDA_CHECK_ERROR                                                       \
    do {                                                                       \
        const cudaError_t err = cudaGetLastError();                            \
        if (err != cudaSuccess) {                                              \
            const char *const err_str = cudaGetErrorString(err);               \
            std::cerr << "Cuda error in " << __FILE__ << ":" << __LINE__ - 1   \
                      << ": " << err_str << " (" << err << ")" << std::endl;   \
            exit(EXIT_FAILURE);                                                \
        }                                                                      \
    } while(0)
/*
 * CUDA HELPER FUNCTIONS ===============================================================================================
 */

// Define the motor currently used for transforming as constant memory
__constant__ float constantMotor[8];

__device__ unsigned int xorshift32(unsigned int state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

/*
 * Takes a list of correspondences and indices and saves the result into outMotors
 */
__global__ void calculateMotors(const float* corr, const unsigned int* indices, float* outMotors, const int iteration, const int trianglesCount, const int correspondenceCount) {
    // 1D Thread ID
    int idx = threadIdx.x;

    // Load the correspondences into shared memory
    extern __shared__ float sharedCorrespondences[];

    // Load using all threads
    for (int i = idx; i < 6*correspondenceCount; i += blockDim.x) {
        sharedCorrespondences[i] = corr[i];
    }

    // If this thread is out of bounds for the triangle calculation: Skip
    if(idx > trianglesCount - 1) {
        return;
    }

    __syncthreads();

    // Generate three indices
    unsigned int id1 = 6*(xorshift32(133*idx) % correspondenceCount);
    unsigned int id2 = 6*(xorshift32(133*idx+1) % correspondenceCount);
    unsigned int id3 = 6*(xorshift32(133*idx+2) % correspondenceCount);

    // Load the current triangle into temp arrays
    float A_src_arr[4] = {sharedCorrespondences[id1], sharedCorrespondences[id1+1], sharedCorrespondences[id1+2], 1.0f};
    float B_src_arr[4] = {sharedCorrespondences[id2], sharedCorrespondences[id2+1], sharedCorrespondences[id2+2], 1.0f};
    float C_src_arr[4] = {sharedCorrespondences[id3], sharedCorrespondences[id3+1], sharedCorrespondences[id3+2], 1.0f};
    float A_tar_arr[4] = {sharedCorrespondences[id1+3], sharedCorrespondences[id1+4], sharedCorrespondences[id1+5], 1.0f};
    float B_tar_arr[4] = {sharedCorrespondences[id2+3], sharedCorrespondences[id2+4], sharedCorrespondences[id2+5], 1.0f};
    float C_tar_arr[4] = {sharedCorrespondences[id3+3], sharedCorrespondences[id3+4], sharedCorrespondences[id3+5], 1.0f};

    // Init the output
    float calculatedMotor[8] = {0.0};



#line 119 "D:/Development/GAAlign/src/optimization/gradient_descent_cuda/gradient_descent_cuda.cug"
#include <math.h>
//#pragma gpc multivector A_src
float A_src[8];
//#pragma gpc multivector A_src_raw
float A_src_raw[2];
//#pragma gpc multivector A_tar
float A_tar[2];
//#pragma gpc multivector B2
float B2[8];
//#pragma gpc multivector B_src
float B_src[8];
//#pragma gpc multivector B_src_raw
float B_src_raw[2];
//#pragma gpc multivector B_tar
float B_tar[2];
//#pragma gpc multivector C2
float C2[8];
//#pragma gpc multivector C3
float C3[4];
//#pragma gpc multivector C_src
float C_src[8];
//#pragma gpc multivector C_src_raw
float C_src_raw[2];
//#pragma gpc multivector C_tar
float C_tar[2];
//#pragma gpc multivector combined_motor
float combined_motor[8];
//#pragma gpc multivector L1
float L1[6];
//#pragma gpc multivector L2
float L2[7];
//#pragma gpc multivector motor_norm
float motor_norm;
//#pragma gpc multivector out_motor
float out_motor[8];
//#pragma gpc multivector P1
float P1[4];
//#pragma gpc multivector P2
float P2[4];
//#pragma gpc multivector VA
float VA[8];
//#pragma gpc multivector VA_norm
float VA_norm;
//#pragma gpc multivector VA_unnormalized
float VA_unnormalized[8];
//#pragma gpc multivector VB
float VB[8];
//#pragma gpc multivector VB_norm
float VB_norm;
//#pragma gpc multivector VB_unnormalized
float VB_unnormalized[8];
//#pragma gpc multivector VC
float VC[7];
//#pragma gpc multivector VC_norm
float VC_norm;
//#pragma gpc multivector VC_unnormalized
float VC_unnormalized[7];

//#pragma gpc multivector_component A_src_raw e0^e1^e2 A_src_raw[0]
A_src_raw[0] = (-A_src_arr[2]);
//#pragma gpc multivector_component A_src_raw e0^e2^e3 A_src_raw[1]
A_src_raw[1] = (-A_src_arr[0]);
//#pragma gpc multivector_component B_src_raw e0^e1^e2 B_src_raw[0]
B_src_raw[0] = (-B_src_arr[2]);
//#pragma gpc multivector_component B_src_raw e0^e2^e3 B_src_raw[1]
B_src_raw[1] = (-B_src_arr[0]);
//#pragma gpc multivector_component C_src_raw e0^e1^e2 C_src_raw[0]
C_src_raw[0] = (-C_src_arr[2]);
//#pragma gpc multivector_component C_src_raw e0^e2^e3 C_src_raw[1]
C_src_raw[1] = (-C_src_arr[0]);
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
//#pragma gpc multivector_component A_src e0 A_src[0]
A_src[0] = ((-(constantMotor[4] * A_src_raw[0])) + (-(constantMotor[5] * A_src_arr[1])) + (-(constantMotor[6] * A_src_raw[1])) + (-(constantMotor[7] * A_src_arr[3]))) * constantMotor[0] + (-((-(constantMotor[6] * A_src_arr[3])) * (-constantMotor[1]))) + (-(constantMotor[5] * A_src_arr[3] * (-constantMotor[2]))) + (-((-(constantMotor[4] * A_src_arr[3])) * (-constantMotor[3]))) + (-((constantMotor[0] * A_src_raw[0] + constantMotor[3] * A_src_arr[3] + (-(constantMotor[5] * A_src_raw[1])) + constantMotor[6] * A_src_arr[1]) * (-constantMotor[4]))) + (-((constantMotor[0] * A_src_arr[1] + (-(constantMotor[2] * A_src_arr[3])) + constantMotor[4] * A_src_raw[1] + (-(constantMotor[6] * A_src_raw[0]))) * (-constantMotor[5]))) + (-((constantMotor[0] * A_src_raw[1] + constantMotor[1] * A_src_arr[3] + (-(constantMotor[4] * A_src_arr[1])) + constantMotor[5] * A_src_raw[0]) * (-constantMotor[6]))) + constantMotor[0] * A_src_arr[3] * constantMotor[7];
//#pragma gpc multivector_component A_src e1 A_src[1]
A_src[1] = (-(constantMotor[6] * A_src_arr[3])) * constantMotor[0] + (-(constantMotor[5] * A_src_arr[3] * (-constantMotor[4]))) + (-((-(constantMotor[4] * A_src_arr[3])) * (-constantMotor[5]))) + (-(constantMotor[0] * A_src_arr[3] * (-constantMotor[6])));
//#pragma gpc multivector_component A_src e2 A_src[2]
A_src[2] = (-(constantMotor[6] * A_src_arr[3])) * (-constantMotor[4]) + constantMotor[5] * A_src_arr[3] * constantMotor[0] + (-((-(constantMotor[4] * A_src_arr[3])) * (-constantMotor[6]))) + constantMotor[0] * A_src_arr[3] * (-constantMotor[5]);
//#pragma gpc multivector_component A_src e3 A_src[3]
A_src[3] = (-(constantMotor[6] * A_src_arr[3])) * (-constantMotor[5]) + constantMotor[5] * A_src_arr[3] * (-constantMotor[6]) + (-(constantMotor[4] * A_src_arr[3])) * constantMotor[0] + (-(constantMotor[0] * A_src_arr[3] * (-constantMotor[4])));
//#pragma gpc multivector_component A_src e0^e1^e2 A_src[4]
A_src[4] = ((-(constantMotor[4] * A_src_raw[0])) + (-(constantMotor[5] * A_src_arr[1])) + (-(constantMotor[6] * A_src_raw[1])) + (-(constantMotor[7] * A_src_arr[3]))) * (-constantMotor[4]) + (-((-(constantMotor[6] * A_src_arr[3])) * (-constantMotor[2]))) + constantMotor[5] * A_src_arr[3] * (-constantMotor[1]) + (-((-(constantMotor[4] * A_src_arr[3])) * constantMotor[7])) + (constantMotor[0] * A_src_raw[0] + constantMotor[3] * A_src_arr[3] + (-(constantMotor[5] * A_src_raw[1])) + constantMotor[6] * A_src_arr[1]) * constantMotor[0] + (-((constantMotor[0] * A_src_arr[1] + (-(constantMotor[2] * A_src_arr[3])) + constantMotor[4] * A_src_raw[1] + (-(constantMotor[6] * A_src_raw[0]))) * (-constantMotor[6]))) + (constantMotor[0] * A_src_raw[1] + constantMotor[1] * A_src_arr[3] + (-(constantMotor[4] * A_src_arr[1])) + constantMotor[5] * A_src_raw[0]) * (-constantMotor[5]) + (-(constantMotor[0] * A_src_arr[3] * (-constantMotor[3])));
//#pragma gpc multivector_component A_src e0^e1^e3 A_src[5]
A_src[5] = ((-(constantMotor[4] * A_src_raw[0])) + (-(constantMotor[5] * A_src_arr[1])) + (-(constantMotor[6] * A_src_raw[1])) + (-(constantMotor[7] * A_src_arr[3]))) * (-constantMotor[5]) + (-((-(constantMotor[6] * A_src_arr[3])) * (-constantMotor[3]))) + constantMotor[5] * A_src_arr[3] * constantMotor[7] + (-(constantMotor[4] * A_src_arr[3])) * (-constantMotor[1]) + (constantMotor[0] * A_src_raw[0] + constantMotor[3] * A_src_arr[3] + (-(constantMotor[5] * A_src_raw[1])) + constantMotor[6] * A_src_arr[1]) * (-constantMotor[6]) + (constantMotor[0] * A_src_arr[1] + (-(constantMotor[2] * A_src_arr[3])) + constantMotor[4] * A_src_raw[1] + (-(constantMotor[6] * A_src_raw[0]))) * constantMotor[0] + (-((constantMotor[0] * A_src_raw[1] + constantMotor[1] * A_src_arr[3] + (-(constantMotor[4] * A_src_arr[1])) + constantMotor[5] * A_src_raw[0]) * (-constantMotor[4]))) + constantMotor[0] * A_src_arr[3] * (-constantMotor[2]);
//#pragma gpc multivector_component A_src e0^e2^e3 A_src[6]
A_src[6] = ((-(constantMotor[4] * A_src_raw[0])) + (-(constantMotor[5] * A_src_arr[1])) + (-(constantMotor[6] * A_src_raw[1])) + (-(constantMotor[7] * A_src_arr[3]))) * (-constantMotor[6]) + (-((-(constantMotor[6] * A_src_arr[3])) * constantMotor[7])) + (-(constantMotor[5] * A_src_arr[3] * (-constantMotor[3]))) + (-(constantMotor[4] * A_src_arr[3])) * (-constantMotor[2]) + (-((constantMotor[0] * A_src_raw[0] + constantMotor[3] * A_src_arr[3] + (-(constantMotor[5] * A_src_raw[1])) + constantMotor[6] * A_src_arr[1]) * (-constantMotor[5]))) + (constantMotor[0] * A_src_arr[1] + (-(constantMotor[2] * A_src_arr[3])) + constantMotor[4] * A_src_raw[1] + (-(constantMotor[6] * A_src_raw[0]))) * (-constantMotor[4]) + (constantMotor[0] * A_src_raw[1] + constantMotor[1] * A_src_arr[3] + (-(constantMotor[4] * A_src_arr[1])) + constantMotor[5] * A_src_raw[0]) * constantMotor[0] + (-(constantMotor[0] * A_src_arr[3] * (-constantMotor[1])));
//#pragma gpc multivector_component A_src e1^e2^e3 A_src[7]
A_src[7] = (-(constantMotor[6] * A_src_arr[3])) * (-constantMotor[6]) + (-(constantMotor[5] * A_src_arr[3] * (-constantMotor[5]))) + (-(constantMotor[4] * A_src_arr[3])) * (-constantMotor[4]) + constantMotor[0] * A_src_arr[3] * constantMotor[0];
//#pragma gpc multivector_component B_src e0 B_src[0]
B_src[0] = ((-(constantMotor[4] * B_src_raw[0])) + (-(constantMotor[5] * B_src_arr[1])) + (-(constantMotor[6] * B_src_raw[1])) + (-(constantMotor[7] * B_src_arr[3]))) * constantMotor[0] + (-((-(constantMotor[6] * B_src_arr[3])) * (-constantMotor[1]))) + (-(constantMotor[5] * B_src_arr[3] * (-constantMotor[2]))) + (-((-(constantMotor[4] * B_src_arr[3])) * (-constantMotor[3]))) + (-((constantMotor[0] * B_src_raw[0] + constantMotor[3] * B_src_arr[3] + (-(constantMotor[5] * B_src_raw[1])) + constantMotor[6] * B_src_arr[1]) * (-constantMotor[4]))) + (-((constantMotor[0] * B_src_arr[1] + (-(constantMotor[2] * B_src_arr[3])) + constantMotor[4] * B_src_raw[1] + (-(constantMotor[6] * B_src_raw[0]))) * (-constantMotor[5]))) + (-((constantMotor[0] * B_src_raw[1] + constantMotor[1] * B_src_arr[3] + (-(constantMotor[4] * B_src_arr[1])) + constantMotor[5] * B_src_raw[0]) * (-constantMotor[6]))) + constantMotor[0] * B_src_arr[3] * constantMotor[7];
//#pragma gpc multivector_component B_src e1 B_src[1]
B_src[1] = (-(constantMotor[6] * B_src_arr[3])) * constantMotor[0] + (-(constantMotor[5] * B_src_arr[3] * (-constantMotor[4]))) + (-((-(constantMotor[4] * B_src_arr[3])) * (-constantMotor[5]))) + (-(constantMotor[0] * B_src_arr[3] * (-constantMotor[6])));
//#pragma gpc multivector_component B_src e2 B_src[2]
B_src[2] = (-(constantMotor[6] * B_src_arr[3])) * (-constantMotor[4]) + constantMotor[5] * B_src_arr[3] * constantMotor[0] + (-((-(constantMotor[4] * B_src_arr[3])) * (-constantMotor[6]))) + constantMotor[0] * B_src_arr[3] * (-constantMotor[5]);
//#pragma gpc multivector_component B_src e3 B_src[3]
B_src[3] = (-(constantMotor[6] * B_src_arr[3])) * (-constantMotor[5]) + constantMotor[5] * B_src_arr[3] * (-constantMotor[6]) + (-(constantMotor[4] * B_src_arr[3])) * constantMotor[0] + (-(constantMotor[0] * B_src_arr[3] * (-constantMotor[4])));
//#pragma gpc multivector_component B_src e0^e1^e2 B_src[4]
B_src[4] = ((-(constantMotor[4] * B_src_raw[0])) + (-(constantMotor[5] * B_src_arr[1])) + (-(constantMotor[6] * B_src_raw[1])) + (-(constantMotor[7] * B_src_arr[3]))) * (-constantMotor[4]) + (-((-(constantMotor[6] * B_src_arr[3])) * (-constantMotor[2]))) + constantMotor[5] * B_src_arr[3] * (-constantMotor[1]) + (-((-(constantMotor[4] * B_src_arr[3])) * constantMotor[7])) + (constantMotor[0] * B_src_raw[0] + constantMotor[3] * B_src_arr[3] + (-(constantMotor[5] * B_src_raw[1])) + constantMotor[6] * B_src_arr[1]) * constantMotor[0] + (-((constantMotor[0] * B_src_arr[1] + (-(constantMotor[2] * B_src_arr[3])) + constantMotor[4] * B_src_raw[1] + (-(constantMotor[6] * B_src_raw[0]))) * (-constantMotor[6]))) + (constantMotor[0] * B_src_raw[1] + constantMotor[1] * B_src_arr[3] + (-(constantMotor[4] * B_src_arr[1])) + constantMotor[5] * B_src_raw[0]) * (-constantMotor[5]) + (-(constantMotor[0] * B_src_arr[3] * (-constantMotor[3])));
//#pragma gpc multivector_component B_src e0^e1^e3 B_src[5]
B_src[5] = ((-(constantMotor[4] * B_src_raw[0])) + (-(constantMotor[5] * B_src_arr[1])) + (-(constantMotor[6] * B_src_raw[1])) + (-(constantMotor[7] * B_src_arr[3]))) * (-constantMotor[5]) + (-((-(constantMotor[6] * B_src_arr[3])) * (-constantMotor[3]))) + constantMotor[5] * B_src_arr[3] * constantMotor[7] + (-(constantMotor[4] * B_src_arr[3])) * (-constantMotor[1]) + (constantMotor[0] * B_src_raw[0] + constantMotor[3] * B_src_arr[3] + (-(constantMotor[5] * B_src_raw[1])) + constantMotor[6] * B_src_arr[1]) * (-constantMotor[6]) + (constantMotor[0] * B_src_arr[1] + (-(constantMotor[2] * B_src_arr[3])) + constantMotor[4] * B_src_raw[1] + (-(constantMotor[6] * B_src_raw[0]))) * constantMotor[0] + (-((constantMotor[0] * B_src_raw[1] + constantMotor[1] * B_src_arr[3] + (-(constantMotor[4] * B_src_arr[1])) + constantMotor[5] * B_src_raw[0]) * (-constantMotor[4]))) + constantMotor[0] * B_src_arr[3] * (-constantMotor[2]);
//#pragma gpc multivector_component B_src e0^e2^e3 B_src[6]
B_src[6] = ((-(constantMotor[4] * B_src_raw[0])) + (-(constantMotor[5] * B_src_arr[1])) + (-(constantMotor[6] * B_src_raw[1])) + (-(constantMotor[7] * B_src_arr[3]))) * (-constantMotor[6]) + (-((-(constantMotor[6] * B_src_arr[3])) * constantMotor[7])) + (-(constantMotor[5] * B_src_arr[3] * (-constantMotor[3]))) + (-(constantMotor[4] * B_src_arr[3])) * (-constantMotor[2]) + (-((constantMotor[0] * B_src_raw[0] + constantMotor[3] * B_src_arr[3] + (-(constantMotor[5] * B_src_raw[1])) + constantMotor[6] * B_src_arr[1]) * (-constantMotor[5]))) + (constantMotor[0] * B_src_arr[1] + (-(constantMotor[2] * B_src_arr[3])) + constantMotor[4] * B_src_raw[1] + (-(constantMotor[6] * B_src_raw[0]))) * (-constantMotor[4]) + (constantMotor[0] * B_src_raw[1] + constantMotor[1] * B_src_arr[3] + (-(constantMotor[4] * B_src_arr[1])) + constantMotor[5] * B_src_raw[0]) * constantMotor[0] + (-(constantMotor[0] * B_src_arr[3] * (-constantMotor[1])));
//#pragma gpc multivector_component B_src e1^e2^e3 B_src[7]
B_src[7] = (-(constantMotor[6] * B_src_arr[3])) * (-constantMotor[6]) + (-(constantMotor[5] * B_src_arr[3] * (-constantMotor[5]))) + (-(constantMotor[4] * B_src_arr[3])) * (-constantMotor[4]) + constantMotor[0] * B_src_arr[3] * constantMotor[0];
//#pragma gpc multivector_component C_src e0 C_src[0]
C_src[0] = ((-(constantMotor[4] * C_src_raw[0])) + (-(constantMotor[5] * C_src_arr[1])) + (-(constantMotor[6] * C_src_raw[1])) + (-(constantMotor[7] * C_src_arr[3]))) * constantMotor[0] + (-((-(constantMotor[6] * C_src_arr[3])) * (-constantMotor[1]))) + (-(constantMotor[5] * C_src_arr[3] * (-constantMotor[2]))) + (-((-(constantMotor[4] * C_src_arr[3])) * (-constantMotor[3]))) + (-((constantMotor[0] * C_src_raw[0] + constantMotor[3] * C_src_arr[3] + (-(constantMotor[5] * C_src_raw[1])) + constantMotor[6] * C_src_arr[1]) * (-constantMotor[4]))) + (-((constantMotor[0] * C_src_arr[1] + (-(constantMotor[2] * C_src_arr[3])) + constantMotor[4] * C_src_raw[1] + (-(constantMotor[6] * C_src_raw[0]))) * (-constantMotor[5]))) + (-((constantMotor[0] * C_src_raw[1] + constantMotor[1] * C_src_arr[3] + (-(constantMotor[4] * C_src_arr[1])) + constantMotor[5] * C_src_raw[0]) * (-constantMotor[6]))) + constantMotor[0] * C_src_arr[3] * constantMotor[7];
//#pragma gpc multivector_component C_src e1 C_src[1]
C_src[1] = (-(constantMotor[6] * C_src_arr[3])) * constantMotor[0] + (-(constantMotor[5] * C_src_arr[3] * (-constantMotor[4]))) + (-((-(constantMotor[4] * C_src_arr[3])) * (-constantMotor[5]))) + (-(constantMotor[0] * C_src_arr[3] * (-constantMotor[6])));
//#pragma gpc multivector_component C_src e2 C_src[2]
C_src[2] = (-(constantMotor[6] * C_src_arr[3])) * (-constantMotor[4]) + constantMotor[5] * C_src_arr[3] * constantMotor[0] + (-((-(constantMotor[4] * C_src_arr[3])) * (-constantMotor[6]))) + constantMotor[0] * C_src_arr[3] * (-constantMotor[5]);
//#pragma gpc multivector_component C_src e3 C_src[3]
C_src[3] = (-(constantMotor[6] * C_src_arr[3])) * (-constantMotor[5]) + constantMotor[5] * C_src_arr[3] * (-constantMotor[6]) + (-(constantMotor[4] * C_src_arr[3])) * constantMotor[0] + (-(constantMotor[0] * C_src_arr[3] * (-constantMotor[4])));
//#pragma gpc multivector_component C_src e0^e1^e2 C_src[4]
C_src[4] = ((-(constantMotor[4] * C_src_raw[0])) + (-(constantMotor[5] * C_src_arr[1])) + (-(constantMotor[6] * C_src_raw[1])) + (-(constantMotor[7] * C_src_arr[3]))) * (-constantMotor[4]) + (-((-(constantMotor[6] * C_src_arr[3])) * (-constantMotor[2]))) + constantMotor[5] * C_src_arr[3] * (-constantMotor[1]) + (-((-(constantMotor[4] * C_src_arr[3])) * constantMotor[7])) + (constantMotor[0] * C_src_raw[0] + constantMotor[3] * C_src_arr[3] + (-(constantMotor[5] * C_src_raw[1])) + constantMotor[6] * C_src_arr[1]) * constantMotor[0] + (-((constantMotor[0] * C_src_arr[1] + (-(constantMotor[2] * C_src_arr[3])) + constantMotor[4] * C_src_raw[1] + (-(constantMotor[6] * C_src_raw[0]))) * (-constantMotor[6]))) + (constantMotor[0] * C_src_raw[1] + constantMotor[1] * C_src_arr[3] + (-(constantMotor[4] * C_src_arr[1])) + constantMotor[5] * C_src_raw[0]) * (-constantMotor[5]) + (-(constantMotor[0] * C_src_arr[3] * (-constantMotor[3])));
//#pragma gpc multivector_component C_src e0^e1^e3 C_src[5]
C_src[5] = ((-(constantMotor[4] * C_src_raw[0])) + (-(constantMotor[5] * C_src_arr[1])) + (-(constantMotor[6] * C_src_raw[1])) + (-(constantMotor[7] * C_src_arr[3]))) * (-constantMotor[5]) + (-((-(constantMotor[6] * C_src_arr[3])) * (-constantMotor[3]))) + constantMotor[5] * C_src_arr[3] * constantMotor[7] + (-(constantMotor[4] * C_src_arr[3])) * (-constantMotor[1]) + (constantMotor[0] * C_src_raw[0] + constantMotor[3] * C_src_arr[3] + (-(constantMotor[5] * C_src_raw[1])) + constantMotor[6] * C_src_arr[1]) * (-constantMotor[6]) + (constantMotor[0] * C_src_arr[1] + (-(constantMotor[2] * C_src_arr[3])) + constantMotor[4] * C_src_raw[1] + (-(constantMotor[6] * C_src_raw[0]))) * constantMotor[0] + (-((constantMotor[0] * C_src_raw[1] + constantMotor[1] * C_src_arr[3] + (-(constantMotor[4] * C_src_arr[1])) + constantMotor[5] * C_src_raw[0]) * (-constantMotor[4]))) + constantMotor[0] * C_src_arr[3] * (-constantMotor[2]);
//#pragma gpc multivector_component C_src e0^e2^e3 C_src[6]
C_src[6] = ((-(constantMotor[4] * C_src_raw[0])) + (-(constantMotor[5] * C_src_arr[1])) + (-(constantMotor[6] * C_src_raw[1])) + (-(constantMotor[7] * C_src_arr[3]))) * (-constantMotor[6]) + (-((-(constantMotor[6] * C_src_arr[3])) * constantMotor[7])) + (-(constantMotor[5] * C_src_arr[3] * (-constantMotor[3]))) + (-(constantMotor[4] * C_src_arr[3])) * (-constantMotor[2]) + (-((constantMotor[0] * C_src_raw[0] + constantMotor[3] * C_src_arr[3] + (-(constantMotor[5] * C_src_raw[1])) + constantMotor[6] * C_src_arr[1]) * (-constantMotor[5]))) + (constantMotor[0] * C_src_arr[1] + (-(constantMotor[2] * C_src_arr[3])) + constantMotor[4] * C_src_raw[1] + (-(constantMotor[6] * C_src_raw[0]))) * (-constantMotor[4]) + (constantMotor[0] * C_src_raw[1] + constantMotor[1] * C_src_arr[3] + (-(constantMotor[4] * C_src_arr[1])) + constantMotor[5] * C_src_raw[0]) * constantMotor[0] + (-(constantMotor[0] * C_src_arr[3] * (-constantMotor[1])));
//#pragma gpc multivector_component C_src e1^e2^e3 C_src[7]
C_src[7] = (-(constantMotor[6] * C_src_arr[3])) * (-constantMotor[6]) + (-(constantMotor[5] * C_src_arr[3] * (-constantMotor[5]))) + (-(constantMotor[4] * C_src_arr[3])) * (-constantMotor[4]) + constantMotor[0] * C_src_arr[3] * constantMotor[0];
//#pragma gpc multivector_component VA_unnormalized 1.0 VA_unnormalized[0]
VA_unnormalized[0] = 1.0 + (-(A_tar_arr[3] * (-A_src[7]) / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7]))))));
//#pragma gpc multivector_component VA_unnormalized e0^e1 VA_unnormalized[1]
VA_unnormalized[1] = A_tar[0] * A_src[2] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))) + A_tar_arr[1] * A_src[3] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))) + (-(A_tar[1] * (-A_src[7]) / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))))) + A_tar_arr[3] * (-A_src[6]) / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7]))));
//#pragma gpc multivector_component VA_unnormalized e0^e2 VA_unnormalized[2]
VA_unnormalized[2] = (-(A_tar[0] * A_src[1] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))))) + A_tar_arr[1] * (-A_src[7]) / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))) + A_tar[1] * A_src[3] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))) + (-(A_tar_arr[3] * (-A_src[5]) / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7]))))));
//#pragma gpc multivector_component VA_unnormalized e0^e3 VA_unnormalized[3]
VA_unnormalized[3] = (-(A_tar[0] * (-A_src[7]) / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))))) + (-(A_tar_arr[1] * A_src[1] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))))) + (-(A_tar[1] * A_src[2] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))))) + A_tar_arr[3] * (-A_src[4]) / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7]))));
//#pragma gpc multivector_component VA_unnormalized e1^e2 VA_unnormalized[4]
VA_unnormalized[4] = A_tar_arr[3] * A_src[3] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7]))));
//#pragma gpc multivector_component VA_unnormalized e1^e3 VA_unnormalized[5]
VA_unnormalized[5] = (-(A_tar_arr[3] * A_src[2] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7]))))));
//#pragma gpc multivector_component VA_unnormalized e2^e3 VA_unnormalized[6]
VA_unnormalized[6] = A_tar_arr[3] * A_src[1] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7]))));
//#pragma gpc multivector_component VA_unnormalized e0^e1^e2^e3 VA_unnormalized[7]
VA_unnormalized[7] = A_tar[0] * A_src[3] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))) + (-(A_tar_arr[1] * A_src[2] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))))) + A_tar[1] * A_src[1] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7])))) + (-(A_tar_arr[3] * A_src[0] / (A_src[1] * A_src[1] + A_src[2] * A_src[2] + A_src[3] * A_src[3] + (-(A_src[7] * (-A_src[7]))))));
//#pragma gpc multivector_component VA_norm 1.0 VA_norm
VA_norm = sqrtf(fabs(VA_unnormalized[0] * VA_unnormalized[0] + (-(VA_unnormalized[4] * (-VA_unnormalized[4]))) + (-(VA_unnormalized[5] * (-VA_unnormalized[5]))) + (-(VA_unnormalized[6] * (-VA_unnormalized[6])))));
//#pragma gpc multivector_component VA 1.0 VA[0]
VA[0] = VA_unnormalized[0] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component VA e0^e1 VA[1]
VA[1] = VA_unnormalized[1] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component VA e0^e2 VA[2]
VA[2] = VA_unnormalized[2] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component VA e0^e3 VA[3]
VA[3] = VA_unnormalized[3] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component VA e1^e2 VA[4]
VA[4] = VA_unnormalized[4] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component VA e1^e3 VA[5]
VA[5] = VA_unnormalized[5] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component VA e2^e3 VA[6]
VA[6] = VA_unnormalized[6] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component VA e0^e1^e2^e3 VA[7]
VA[7] = VA_unnormalized[7] * VA_norm / (VA_norm * VA_norm);
//#pragma gpc multivector_component B2 e0 B2[0]
B2[0] = (VA[0] * B_src[0] + VA[1] * B_src[1] + VA[2] * B_src[2] + VA[3] * B_src[3] + (-(VA[4] * B_src[4])) + (-(VA[5] * B_src[5])) + (-(VA[6] * B_src[6])) + (-(VA[7] * B_src[7]))) * VA[0] + (-((VA[0] * B_src[1] + VA[4] * B_src[2] + VA[5] * B_src[3] + (-(VA[6] * B_src[7]))) * (-VA[1]))) + (-((VA[0] * B_src[2] + (-(VA[4] * B_src[1])) + VA[5] * B_src[7] + VA[6] * B_src[3]) * (-VA[2]))) + (-((VA[0] * B_src[3] + (-(VA[4] * B_src[7])) + (-(VA[5] * B_src[1])) + (-(VA[6] * B_src[2]))) * (-VA[3]))) + (-((VA[0] * B_src[4] + VA[1] * B_src[2] + (-(VA[2] * B_src[1])) + VA[3] * B_src[7] + VA[4] * B_src[0] + (-(VA[5] * B_src[6])) + VA[6] * B_src[5] + VA[7] * B_src[3]) * (-VA[4]))) + (-((VA[0] * B_src[5] + VA[1] * B_src[3] + (-(VA[2] * B_src[7])) + (-(VA[3] * B_src[1])) + VA[4] * B_src[6] + VA[5] * B_src[0] + (-(VA[6] * B_src[4])) + (-(VA[7] * B_src[2]))) * (-VA[5]))) + (-((VA[0] * B_src[6] + VA[1] * B_src[7] + VA[2] * B_src[3] + (-(VA[3] * B_src[2])) + (-(VA[4] * B_src[5])) + VA[5] * B_src[4] + VA[6] * B_src[0] + VA[7] * B_src[1]) * (-VA[6]))) + (VA[0] * B_src[7] + VA[4] * B_src[3] + (-(VA[5] * B_src[2])) + VA[6] * B_src[1]) * VA[7];
//#pragma gpc multivector_component B2 e1 B2[1]
B2[1] = (VA[0] * B_src[1] + VA[4] * B_src[2] + VA[5] * B_src[3] + (-(VA[6] * B_src[7]))) * VA[0] + (-((VA[0] * B_src[2] + (-(VA[4] * B_src[1])) + VA[5] * B_src[7] + VA[6] * B_src[3]) * (-VA[4]))) + (-((VA[0] * B_src[3] + (-(VA[4] * B_src[7])) + (-(VA[5] * B_src[1])) + (-(VA[6] * B_src[2]))) * (-VA[5]))) + (-((VA[0] * B_src[7] + VA[4] * B_src[3] + (-(VA[5] * B_src[2])) + VA[6] * B_src[1]) * (-VA[6])));
//#pragma gpc multivector_component B2 e2 B2[2]
B2[2] = (VA[0] * B_src[1] + VA[4] * B_src[2] + VA[5] * B_src[3] + (-(VA[6] * B_src[7]))) * (-VA[4]) + (VA[0] * B_src[2] + (-(VA[4] * B_src[1])) + VA[5] * B_src[7] + VA[6] * B_src[3]) * VA[0] + (-((VA[0] * B_src[3] + (-(VA[4] * B_src[7])) + (-(VA[5] * B_src[1])) + (-(VA[6] * B_src[2]))) * (-VA[6]))) + (VA[0] * B_src[7] + VA[4] * B_src[3] + (-(VA[5] * B_src[2])) + VA[6] * B_src[1]) * (-VA[5]);
//#pragma gpc multivector_component B2 e3 B2[3]
B2[3] = (VA[0] * B_src[1] + VA[4] * B_src[2] + VA[5] * B_src[3] + (-(VA[6] * B_src[7]))) * (-VA[5]) + (VA[0] * B_src[2] + (-(VA[4] * B_src[1])) + VA[5] * B_src[7] + VA[6] * B_src[3]) * (-VA[6]) + (VA[0] * B_src[3] + (-(VA[4] * B_src[7])) + (-(VA[5] * B_src[1])) + (-(VA[6] * B_src[2]))) * VA[0] + (-((VA[0] * B_src[7] + VA[4] * B_src[3] + (-(VA[5] * B_src[2])) + VA[6] * B_src[1]) * (-VA[4])));
//#pragma gpc multivector_component B2 e0^e1^e2 B2[4]
B2[4] = (VA[0] * B_src[0] + VA[1] * B_src[1] + VA[2] * B_src[2] + VA[3] * B_src[3] + (-(VA[4] * B_src[4])) + (-(VA[5] * B_src[5])) + (-(VA[6] * B_src[6])) + (-(VA[7] * B_src[7]))) * (-VA[4]) + (-((VA[0] * B_src[1] + VA[4] * B_src[2] + VA[5] * B_src[3] + (-(VA[6] * B_src[7]))) * (-VA[2]))) + (VA[0] * B_src[2] + (-(VA[4] * B_src[1])) + VA[5] * B_src[7] + VA[6] * B_src[3]) * (-VA[1]) + (-((VA[0] * B_src[3] + (-(VA[4] * B_src[7])) + (-(VA[5] * B_src[1])) + (-(VA[6] * B_src[2]))) * VA[7])) + (VA[0] * B_src[4] + VA[1] * B_src[2] + (-(VA[2] * B_src[1])) + VA[3] * B_src[7] + VA[4] * B_src[0] + (-(VA[5] * B_src[6])) + VA[6] * B_src[5] + VA[7] * B_src[3]) * VA[0] + (-((VA[0] * B_src[5] + VA[1] * B_src[3] + (-(VA[2] * B_src[7])) + (-(VA[3] * B_src[1])) + VA[4] * B_src[6] + VA[5] * B_src[0] + (-(VA[6] * B_src[4])) + (-(VA[7] * B_src[2]))) * (-VA[6]))) + (VA[0] * B_src[6] + VA[1] * B_src[7] + VA[2] * B_src[3] + (-(VA[3] * B_src[2])) + (-(VA[4] * B_src[5])) + VA[5] * B_src[4] + VA[6] * B_src[0] + VA[7] * B_src[1]) * (-VA[5]) + (-((VA[0] * B_src[7] + VA[4] * B_src[3] + (-(VA[5] * B_src[2])) + VA[6] * B_src[1]) * (-VA[3])));
//#pragma gpc multivector_component B2 e0^e1^e3 B2[5]
B2[5] = (VA[0] * B_src[0] + VA[1] * B_src[1] + VA[2] * B_src[2] + VA[3] * B_src[3] + (-(VA[4] * B_src[4])) + (-(VA[5] * B_src[5])) + (-(VA[6] * B_src[6])) + (-(VA[7] * B_src[7]))) * (-VA[5]) + (-((VA[0] * B_src[1] + VA[4] * B_src[2] + VA[5] * B_src[3] + (-(VA[6] * B_src[7]))) * (-VA[3]))) + (VA[0] * B_src[2] + (-(VA[4] * B_src[1])) + VA[5] * B_src[7] + VA[6] * B_src[3]) * VA[7] + (VA[0] * B_src[3] + (-(VA[4] * B_src[7])) + (-(VA[5] * B_src[1])) + (-(VA[6] * B_src[2]))) * (-VA[1]) + (VA[0] * B_src[4] + VA[1] * B_src[2] + (-(VA[2] * B_src[1])) + VA[3] * B_src[7] + VA[4] * B_src[0] + (-(VA[5] * B_src[6])) + VA[6] * B_src[5] + VA[7] * B_src[3]) * (-VA[6]) + (VA[0] * B_src[5] + VA[1] * B_src[3] + (-(VA[2] * B_src[7])) + (-(VA[3] * B_src[1])) + VA[4] * B_src[6] + VA[5] * B_src[0] + (-(VA[6] * B_src[4])) + (-(VA[7] * B_src[2]))) * VA[0] + (-((VA[0] * B_src[6] + VA[1] * B_src[7] + VA[2] * B_src[3] + (-(VA[3] * B_src[2])) + (-(VA[4] * B_src[5])) + VA[5] * B_src[4] + VA[6] * B_src[0] + VA[7] * B_src[1]) * (-VA[4]))) + (VA[0] * B_src[7] + VA[4] * B_src[3] + (-(VA[5] * B_src[2])) + VA[6] * B_src[1]) * (-VA[2]);
//#pragma gpc multivector_component B2 e0^e2^e3 B2[6]
B2[6] = (VA[0] * B_src[0] + VA[1] * B_src[1] + VA[2] * B_src[2] + VA[3] * B_src[3] + (-(VA[4] * B_src[4])) + (-(VA[5] * B_src[5])) + (-(VA[6] * B_src[6])) + (-(VA[7] * B_src[7]))) * (-VA[6]) + (-((VA[0] * B_src[1] + VA[4] * B_src[2] + VA[5] * B_src[3] + (-(VA[6] * B_src[7]))) * VA[7])) + (-((VA[0] * B_src[2] + (-(VA[4] * B_src[1])) + VA[5] * B_src[7] + VA[6] * B_src[3]) * (-VA[3]))) + (VA[0] * B_src[3] + (-(VA[4] * B_src[7])) + (-(VA[5] * B_src[1])) + (-(VA[6] * B_src[2]))) * (-VA[2]) + (-((VA[0] * B_src[4] + VA[1] * B_src[2] + (-(VA[2] * B_src[1])) + VA[3] * B_src[7] + VA[4] * B_src[0] + (-(VA[5] * B_src[6])) + VA[6] * B_src[5] + VA[7] * B_src[3]) * (-VA[5]))) + (VA[0] * B_src[5] + VA[1] * B_src[3] + (-(VA[2] * B_src[7])) + (-(VA[3] * B_src[1])) + VA[4] * B_src[6] + VA[5] * B_src[0] + (-(VA[6] * B_src[4])) + (-(VA[7] * B_src[2]))) * (-VA[4]) + (VA[0] * B_src[6] + VA[1] * B_src[7] + VA[2] * B_src[3] + (-(VA[3] * B_src[2])) + (-(VA[4] * B_src[5])) + VA[5] * B_src[4] + VA[6] * B_src[0] + VA[7] * B_src[1]) * VA[0] + (-((VA[0] * B_src[7] + VA[4] * B_src[3] + (-(VA[5] * B_src[2])) + VA[6] * B_src[1]) * (-VA[1])));
//#pragma gpc multivector_component B2 e1^e2^e3 B2[7]
B2[7] = (VA[0] * B_src[1] + VA[4] * B_src[2] + VA[5] * B_src[3] + (-(VA[6] * B_src[7]))) * (-VA[6]) + (-((VA[0] * B_src[2] + (-(VA[4] * B_src[1])) + VA[5] * B_src[7] + VA[6] * B_src[3]) * (-VA[5]))) + (VA[0] * B_src[3] + (-(VA[4] * B_src[7])) + (-(VA[5] * B_src[1])) + (-(VA[6] * B_src[2]))) * (-VA[4]) + (VA[0] * B_src[7] + VA[4] * B_src[3] + (-(VA[5] * B_src[2])) + VA[6] * B_src[1]) * VA[0];
//#pragma gpc multivector_component C2 e0 C2[0]
C2[0] = (VA[0] * C_src[0] + VA[1] * C_src[1] + VA[2] * C_src[2] + VA[3] * C_src[3] + (-(VA[4] * C_src[4])) + (-(VA[5] * C_src[5])) + (-(VA[6] * C_src[6])) + (-(VA[7] * C_src[7]))) * VA[0] + (-((VA[0] * C_src[1] + VA[4] * C_src[2] + VA[5] * C_src[3] + (-(VA[6] * C_src[7]))) * (-VA[1]))) + (-((VA[0] * C_src[2] + (-(VA[4] * C_src[1])) + VA[5] * C_src[7] + VA[6] * C_src[3]) * (-VA[2]))) + (-((VA[0] * C_src[3] + (-(VA[4] * C_src[7])) + (-(VA[5] * C_src[1])) + (-(VA[6] * C_src[2]))) * (-VA[3]))) + (-((VA[0] * C_src[4] + VA[1] * C_src[2] + (-(VA[2] * C_src[1])) + VA[3] * C_src[7] + VA[4] * C_src[0] + (-(VA[5] * C_src[6])) + VA[6] * C_src[5] + VA[7] * C_src[3]) * (-VA[4]))) + (-((VA[0] * C_src[5] + VA[1] * C_src[3] + (-(VA[2] * C_src[7])) + (-(VA[3] * C_src[1])) + VA[4] * C_src[6] + VA[5] * C_src[0] + (-(VA[6] * C_src[4])) + (-(VA[7] * C_src[2]))) * (-VA[5]))) + (-((VA[0] * C_src[6] + VA[1] * C_src[7] + VA[2] * C_src[3] + (-(VA[3] * C_src[2])) + (-(VA[4] * C_src[5])) + VA[5] * C_src[4] + VA[6] * C_src[0] + VA[7] * C_src[1]) * (-VA[6]))) + (VA[0] * C_src[7] + VA[4] * C_src[3] + (-(VA[5] * C_src[2])) + VA[6] * C_src[1]) * VA[7];
//#pragma gpc multivector_component C2 e1 C2[1]
C2[1] = (VA[0] * C_src[1] + VA[4] * C_src[2] + VA[5] * C_src[3] + (-(VA[6] * C_src[7]))) * VA[0] + (-((VA[0] * C_src[2] + (-(VA[4] * C_src[1])) + VA[5] * C_src[7] + VA[6] * C_src[3]) * (-VA[4]))) + (-((VA[0] * C_src[3] + (-(VA[4] * C_src[7])) + (-(VA[5] * C_src[1])) + (-(VA[6] * C_src[2]))) * (-VA[5]))) + (-((VA[0] * C_src[7] + VA[4] * C_src[3] + (-(VA[5] * C_src[2])) + VA[6] * C_src[1]) * (-VA[6])));
//#pragma gpc multivector_component C2 e2 C2[2]
C2[2] = (VA[0] * C_src[1] + VA[4] * C_src[2] + VA[5] * C_src[3] + (-(VA[6] * C_src[7]))) * (-VA[4]) + (VA[0] * C_src[2] + (-(VA[4] * C_src[1])) + VA[5] * C_src[7] + VA[6] * C_src[3]) * VA[0] + (-((VA[0] * C_src[3] + (-(VA[4] * C_src[7])) + (-(VA[5] * C_src[1])) + (-(VA[6] * C_src[2]))) * (-VA[6]))) + (VA[0] * C_src[7] + VA[4] * C_src[3] + (-(VA[5] * C_src[2])) + VA[6] * C_src[1]) * (-VA[5]);
//#pragma gpc multivector_component C2 e3 C2[3]
C2[3] = (VA[0] * C_src[1] + VA[4] * C_src[2] + VA[5] * C_src[3] + (-(VA[6] * C_src[7]))) * (-VA[5]) + (VA[0] * C_src[2] + (-(VA[4] * C_src[1])) + VA[5] * C_src[7] + VA[6] * C_src[3]) * (-VA[6]) + (VA[0] * C_src[3] + (-(VA[4] * C_src[7])) + (-(VA[5] * C_src[1])) + (-(VA[6] * C_src[2]))) * VA[0] + (-((VA[0] * C_src[7] + VA[4] * C_src[3] + (-(VA[5] * C_src[2])) + VA[6] * C_src[1]) * (-VA[4])));
//#pragma gpc multivector_component C2 e0^e1^e2 C2[4]
C2[4] = (VA[0] * C_src[0] + VA[1] * C_src[1] + VA[2] * C_src[2] + VA[3] * C_src[3] + (-(VA[4] * C_src[4])) + (-(VA[5] * C_src[5])) + (-(VA[6] * C_src[6])) + (-(VA[7] * C_src[7]))) * (-VA[4]) + (-((VA[0] * C_src[1] + VA[4] * C_src[2] + VA[5] * C_src[3] + (-(VA[6] * C_src[7]))) * (-VA[2]))) + (VA[0] * C_src[2] + (-(VA[4] * C_src[1])) + VA[5] * C_src[7] + VA[6] * C_src[3]) * (-VA[1]) + (-((VA[0] * C_src[3] + (-(VA[4] * C_src[7])) + (-(VA[5] * C_src[1])) + (-(VA[6] * C_src[2]))) * VA[7])) + (VA[0] * C_src[4] + VA[1] * C_src[2] + (-(VA[2] * C_src[1])) + VA[3] * C_src[7] + VA[4] * C_src[0] + (-(VA[5] * C_src[6])) + VA[6] * C_src[5] + VA[7] * C_src[3]) * VA[0] + (-((VA[0] * C_src[5] + VA[1] * C_src[3] + (-(VA[2] * C_src[7])) + (-(VA[3] * C_src[1])) + VA[4] * C_src[6] + VA[5] * C_src[0] + (-(VA[6] * C_src[4])) + (-(VA[7] * C_src[2]))) * (-VA[6]))) + (VA[0] * C_src[6] + VA[1] * C_src[7] + VA[2] * C_src[3] + (-(VA[3] * C_src[2])) + (-(VA[4] * C_src[5])) + VA[5] * C_src[4] + VA[6] * C_src[0] + VA[7] * C_src[1]) * (-VA[5]) + (-((VA[0] * C_src[7] + VA[4] * C_src[3] + (-(VA[5] * C_src[2])) + VA[6] * C_src[1]) * (-VA[3])));
//#pragma gpc multivector_component C2 e0^e1^e3 C2[5]
C2[5] = (VA[0] * C_src[0] + VA[1] * C_src[1] + VA[2] * C_src[2] + VA[3] * C_src[3] + (-(VA[4] * C_src[4])) + (-(VA[5] * C_src[5])) + (-(VA[6] * C_src[6])) + (-(VA[7] * C_src[7]))) * (-VA[5]) + (-((VA[0] * C_src[1] + VA[4] * C_src[2] + VA[5] * C_src[3] + (-(VA[6] * C_src[7]))) * (-VA[3]))) + (VA[0] * C_src[2] + (-(VA[4] * C_src[1])) + VA[5] * C_src[7] + VA[6] * C_src[3]) * VA[7] + (VA[0] * C_src[3] + (-(VA[4] * C_src[7])) + (-(VA[5] * C_src[1])) + (-(VA[6] * C_src[2]))) * (-VA[1]) + (VA[0] * C_src[4] + VA[1] * C_src[2] + (-(VA[2] * C_src[1])) + VA[3] * C_src[7] + VA[4] * C_src[0] + (-(VA[5] * C_src[6])) + VA[6] * C_src[5] + VA[7] * C_src[3]) * (-VA[6]) + (VA[0] * C_src[5] + VA[1] * C_src[3] + (-(VA[2] * C_src[7])) + (-(VA[3] * C_src[1])) + VA[4] * C_src[6] + VA[5] * C_src[0] + (-(VA[6] * C_src[4])) + (-(VA[7] * C_src[2]))) * VA[0] + (-((VA[0] * C_src[6] + VA[1] * C_src[7] + VA[2] * C_src[3] + (-(VA[3] * C_src[2])) + (-(VA[4] * C_src[5])) + VA[5] * C_src[4] + VA[6] * C_src[0] + VA[7] * C_src[1]) * (-VA[4]))) + (VA[0] * C_src[7] + VA[4] * C_src[3] + (-(VA[5] * C_src[2])) + VA[6] * C_src[1]) * (-VA[2]);
//#pragma gpc multivector_component C2 e0^e2^e3 C2[6]
C2[6] = (VA[0] * C_src[0] + VA[1] * C_src[1] + VA[2] * C_src[2] + VA[3] * C_src[3] + (-(VA[4] * C_src[4])) + (-(VA[5] * C_src[5])) + (-(VA[6] * C_src[6])) + (-(VA[7] * C_src[7]))) * (-VA[6]) + (-((VA[0] * C_src[1] + VA[4] * C_src[2] + VA[5] * C_src[3] + (-(VA[6] * C_src[7]))) * VA[7])) + (-((VA[0] * C_src[2] + (-(VA[4] * C_src[1])) + VA[5] * C_src[7] + VA[6] * C_src[3]) * (-VA[3]))) + (VA[0] * C_src[3] + (-(VA[4] * C_src[7])) + (-(VA[5] * C_src[1])) + (-(VA[6] * C_src[2]))) * (-VA[2]) + (-((VA[0] * C_src[4] + VA[1] * C_src[2] + (-(VA[2] * C_src[1])) + VA[3] * C_src[7] + VA[4] * C_src[0] + (-(VA[5] * C_src[6])) + VA[6] * C_src[5] + VA[7] * C_src[3]) * (-VA[5]))) + (VA[0] * C_src[5] + VA[1] * C_src[3] + (-(VA[2] * C_src[7])) + (-(VA[3] * C_src[1])) + VA[4] * C_src[6] + VA[5] * C_src[0] + (-(VA[6] * C_src[4])) + (-(VA[7] * C_src[2]))) * (-VA[4]) + (VA[0] * C_src[6] + VA[1] * C_src[7] + VA[2] * C_src[3] + (-(VA[3] * C_src[2])) + (-(VA[4] * C_src[5])) + VA[5] * C_src[4] + VA[6] * C_src[0] + VA[7] * C_src[1]) * VA[0] + (-((VA[0] * C_src[7] + VA[4] * C_src[3] + (-(VA[5] * C_src[2])) + VA[6] * C_src[1]) * (-VA[1])));
//#pragma gpc multivector_component C2 e1^e2^e3 C2[7]
C2[7] = (VA[0] * C_src[1] + VA[4] * C_src[2] + VA[5] * C_src[3] + (-(VA[6] * C_src[7]))) * (-VA[6]) + (-((VA[0] * C_src[2] + (-(VA[4] * C_src[1])) + VA[5] * C_src[7] + VA[6] * C_src[3]) * (-VA[5]))) + (VA[0] * C_src[3] + (-(VA[4] * C_src[7])) + (-(VA[5] * C_src[1])) + (-(VA[6] * C_src[2]))) * (-VA[4]) + (VA[0] * C_src[7] + VA[4] * C_src[3] + (-(VA[5] * C_src[2])) + VA[6] * C_src[1]) * VA[0];
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
//#pragma gpc multivector_component L2 1.0 L2[0]
L2[0] = A_tar_arr[3] * B2[0] + (-((-A_tar[1]) * (-B2[1]))) + A_tar_arr[1] * B2[2] + (-((-A_tar[0]) * (-B2[3])));
//#pragma gpc multivector_component L2 e0^e1 L2[1]
L2[1] = A_tar_arr[1] * (-B2[4]) + (-((-A_tar[0]) * B2[5]));
//#pragma gpc multivector_component L2 e0^e2 L2[2]
L2[2] = (-((-A_tar[1]) * (-B2[4]) + (-((-A_tar[0]) * (-B2[6])))));
//#pragma gpc multivector_component L2 e0^e3 L2[3]
L2[3] = (-A_tar[1]) * B2[5] + (-(A_tar_arr[1] * (-B2[6])));
//#pragma gpc multivector_component L2 e1^e2 L2[4]
L2[4] = A_tar_arr[3] * (-B2[4]) + (-((-A_tar[0]) * B2[7]));
//#pragma gpc multivector_component L2 e1^e3 L2[5]
L2[5] = (-(A_tar_arr[3] * B2[5] + (-(A_tar_arr[1] * B2[7]))));
//#pragma gpc multivector_component L2 e2^e3 L2[6]
L2[6] = A_tar_arr[3] * (-B2[6]) + (-((-A_tar[1]) * B2[7]));
//#pragma gpc multivector_component VB_unnormalized 1.0 VB_unnormalized[0]
VB_unnormalized[0] = 1.0 + (-(L1[3] * (-L2[4]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))))) + (-(L1[4] * (-L2[5]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))))) + (-(L1[5] * (-L2[6]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6]))))));
//#pragma gpc multivector_component VB_unnormalized e0^e1 VB_unnormalized[1]
VB_unnormalized[1] = L1[0] * L2[0] / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + (-(L1[1] * (-L2[4]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))))) + (-(L1[2] * (-L2[5]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))))) + L1[3] * (-L2[2]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + L1[4] * (-L2[3]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6]))));
//#pragma gpc multivector_component VB_unnormalized e0^e2 VB_unnormalized[2]
VB_unnormalized[2] = L1[0] * (-L2[4]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + L1[1] * L2[0] / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + (-(L1[2] * (-L2[6]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))))) + (-(L1[3] * (-L2[1]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))))) + L1[5] * (-L2[3]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6]))));
//#pragma gpc multivector_component VB_unnormalized e0^e3 VB_unnormalized[3]
VB_unnormalized[3] = L1[0] * (-L2[5]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + L1[1] * (-L2[6]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + L1[2] * L2[0] / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + (-(L1[4] * (-L2[1]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))))) + (-(L1[5] * (-L2[2]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6]))))));
//#pragma gpc multivector_component VB_unnormalized e1^e2 VB_unnormalized[4]
VB_unnormalized[4] = L1[3] * L2[0] / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + (-(L1[4] * (-L2[6]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))))) + L1[5] * (-L2[5]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6]))));
//#pragma gpc multivector_component VB_unnormalized e1^e3 VB_unnormalized[5]
VB_unnormalized[5] = L1[3] * (-L2[6]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + L1[4] * L2[0] / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + (-(L1[5] * (-L2[4]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6]))))));
//#pragma gpc multivector_component VB_unnormalized e2^e3 VB_unnormalized[6]
VB_unnormalized[6] = (-(L1[3] * (-L2[5]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))))) + L1[4] * (-L2[4]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + L1[5] * L2[0] / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6]))));
//#pragma gpc multivector_component VB_unnormalized e0^e1^e2^e3 VB_unnormalized[7]
VB_unnormalized[7] = L1[0] * (-L2[6]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + (-(L1[1] * (-L2[5]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))))) + L1[2] * (-L2[4]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + L1[3] * (-L2[3]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))) + (-(L1[4] * (-L2[2]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6])))))) + L1[5] * (-L2[1]) / (L2[0] * L2[0] + (-(L2[4] * (-L2[4]))) + (-(L2[5] * (-L2[5]))) + (-(L2[6] * (-L2[6]))));
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
C3[0] = (VB[0] * C2[0] + VB[1] * C2[1] + VB[2] * C2[2] + VB[3] * C2[3] + (-(VB[4] * C2[4])) + (-(VB[5] * C2[5])) + (-(VB[6] * C2[6])) + (-(VB[7] * C2[7]))) * (-VB[4]) + (-((VB[0] * C2[1] + VB[4] * C2[2] + VB[5] * C2[3] + (-(VB[6] * C2[7]))) * (-VB[2]))) + (VB[0] * C2[2] + (-(VB[4] * C2[1])) + VB[5] * C2[7] + VB[6] * C2[3]) * (-VB[1]) + (-((VB[0] * C2[3] + (-(VB[4] * C2[7])) + (-(VB[5] * C2[1])) + (-(VB[6] * C2[2]))) * VB[7])) + (VB[0] * C2[4] + VB[1] * C2[2] + (-(VB[2] * C2[1])) + VB[3] * C2[7] + VB[4] * C2[0] + (-(VB[5] * C2[6])) + VB[6] * C2[5] + VB[7] * C2[3]) * VB[0] + (-((VB[0] * C2[5] + VB[1] * C2[3] + (-(VB[2] * C2[7])) + (-(VB[3] * C2[1])) + VB[4] * C2[6] + VB[5] * C2[0] + (-(VB[6] * C2[4])) + (-(VB[7] * C2[2]))) * (-VB[6]))) + (VB[0] * C2[6] + VB[1] * C2[7] + VB[2] * C2[3] + (-(VB[3] * C2[2])) + (-(VB[4] * C2[5])) + VB[5] * C2[4] + VB[6] * C2[0] + VB[7] * C2[1]) * (-VB[5]) + (-((VB[0] * C2[7] + VB[4] * C2[3] + (-(VB[5] * C2[2])) + VB[6] * C2[1]) * (-VB[3])));
//#pragma gpc multivector_component C3 e0^e1^e3 C3[1]
C3[1] = (VB[0] * C2[0] + VB[1] * C2[1] + VB[2] * C2[2] + VB[3] * C2[3] + (-(VB[4] * C2[4])) + (-(VB[5] * C2[5])) + (-(VB[6] * C2[6])) + (-(VB[7] * C2[7]))) * (-VB[5]) + (-((VB[0] * C2[1] + VB[4] * C2[2] + VB[5] * C2[3] + (-(VB[6] * C2[7]))) * (-VB[3]))) + (VB[0] * C2[2] + (-(VB[4] * C2[1])) + VB[5] * C2[7] + VB[6] * C2[3]) * VB[7] + (VB[0] * C2[3] + (-(VB[4] * C2[7])) + (-(VB[5] * C2[1])) + (-(VB[6] * C2[2]))) * (-VB[1]) + (VB[0] * C2[4] + VB[1] * C2[2] + (-(VB[2] * C2[1])) + VB[3] * C2[7] + VB[4] * C2[0] + (-(VB[5] * C2[6])) + VB[6] * C2[5] + VB[7] * C2[3]) * (-VB[6]) + (VB[0] * C2[5] + VB[1] * C2[3] + (-(VB[2] * C2[7])) + (-(VB[3] * C2[1])) + VB[4] * C2[6] + VB[5] * C2[0] + (-(VB[6] * C2[4])) + (-(VB[7] * C2[2]))) * VB[0] + (-((VB[0] * C2[6] + VB[1] * C2[7] + VB[2] * C2[3] + (-(VB[3] * C2[2])) + (-(VB[4] * C2[5])) + VB[5] * C2[4] + VB[6] * C2[0] + VB[7] * C2[1]) * (-VB[4]))) + (VB[0] * C2[7] + VB[4] * C2[3] + (-(VB[5] * C2[2])) + VB[6] * C2[1]) * (-VB[2]);
//#pragma gpc multivector_component C3 e0^e2^e3 C3[2]
C3[2] = (VB[0] * C2[0] + VB[1] * C2[1] + VB[2] * C2[2] + VB[3] * C2[3] + (-(VB[4] * C2[4])) + (-(VB[5] * C2[5])) + (-(VB[6] * C2[6])) + (-(VB[7] * C2[7]))) * (-VB[6]) + (-((VB[0] * C2[1] + VB[4] * C2[2] + VB[5] * C2[3] + (-(VB[6] * C2[7]))) * VB[7])) + (-((VB[0] * C2[2] + (-(VB[4] * C2[1])) + VB[5] * C2[7] + VB[6] * C2[3]) * (-VB[3]))) + (VB[0] * C2[3] + (-(VB[4] * C2[7])) + (-(VB[5] * C2[1])) + (-(VB[6] * C2[2]))) * (-VB[2]) + (-((VB[0] * C2[4] + VB[1] * C2[2] + (-(VB[2] * C2[1])) + VB[3] * C2[7] + VB[4] * C2[0] + (-(VB[5] * C2[6])) + VB[6] * C2[5] + VB[7] * C2[3]) * (-VB[5]))) + (VB[0] * C2[5] + VB[1] * C2[3] + (-(VB[2] * C2[7])) + (-(VB[3] * C2[1])) + VB[4] * C2[6] + VB[5] * C2[0] + (-(VB[6] * C2[4])) + (-(VB[7] * C2[2]))) * (-VB[4]) + (VB[0] * C2[6] + VB[1] * C2[7] + VB[2] * C2[3] + (-(VB[3] * C2[2])) + (-(VB[4] * C2[5])) + VB[5] * C2[4] + VB[6] * C2[0] + VB[7] * C2[1]) * VB[0] + (-((VB[0] * C2[7] + VB[4] * C2[3] + (-(VB[5] * C2[2])) + VB[6] * C2[1]) * (-VB[1])));
//#pragma gpc multivector_component C3 e1^e2^e3 C3[3]
C3[3] = (VB[0] * C2[1] + VB[4] * C2[2] + VB[5] * C2[3] + (-(VB[6] * C2[7]))) * (-VB[6]) + (-((VB[0] * C2[2] + (-(VB[4] * C2[1])) + VB[5] * C2[7] + VB[6] * C2[3]) * (-VB[5]))) + (VB[0] * C2[3] + (-(VB[4] * C2[7])) + (-(VB[5] * C2[1])) + (-(VB[6] * C2[2]))) * (-VB[4]) + (VB[0] * C2[7] + VB[4] * C2[3] + (-(VB[5] * C2[2])) + VB[6] * C2[1]) * VB[0];
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
combined_motor[0] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[0] + (-((VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[4])) + (-((VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[5])) + (-((VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[6]));
//#pragma gpc multivector_component combined_motor e0^e1 combined_motor[1]
combined_motor[1] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[1] + (VC[0] * VB[1] + VC[1] * VB[0] + (-(VC[2] * VB[4])) + (-(VC[3] * VB[5])) + VC[4] * VB[2] + VC[5] * VB[3] + (-(VC[6] * VB[7]))) * VA[0] + (-((VC[0] * VB[2] + VC[1] * VB[4] + VC[2] * VB[0] + (-(VC[3] * VB[6])) + (-(VC[4] * VB[1])) + VC[5] * VB[7] + VC[6] * VB[3]) * VA[4])) + (-((VC[0] * VB[3] + VC[1] * VB[5] + VC[2] * VB[6] + VC[3] * VB[0] + (-(VC[4] * VB[7])) + (-(VC[5] * VB[1])) + (-(VC[6] * VB[2]))) * VA[5])) + (VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[2] + (VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[3] + (-((VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[7])) + (-((VC[0] * VB[7] + VC[1] * VB[6] + (-(VC[2] * VB[5])) + VC[3] * VB[4] + VC[4] * VB[3] + (-(VC[5] * VB[2])) + VC[6] * VB[1]) * VA[6]));
//#pragma gpc multivector_component combined_motor e0^e2 combined_motor[2]
combined_motor[2] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[2] + (VC[0] * VB[1] + VC[1] * VB[0] + (-(VC[2] * VB[4])) + (-(VC[3] * VB[5])) + VC[4] * VB[2] + VC[5] * VB[3] + (-(VC[6] * VB[7]))) * VA[4] + (VC[0] * VB[2] + VC[1] * VB[4] + VC[2] * VB[0] + (-(VC[3] * VB[6])) + (-(VC[4] * VB[1])) + VC[5] * VB[7] + VC[6] * VB[3]) * VA[0] + (-((VC[0] * VB[3] + VC[1] * VB[5] + VC[2] * VB[6] + VC[3] * VB[0] + (-(VC[4] * VB[7])) + (-(VC[5] * VB[1])) + (-(VC[6] * VB[2]))) * VA[6])) + (-((VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[1])) + (VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[7] + (VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[3] + (VC[0] * VB[7] + VC[1] * VB[6] + (-(VC[2] * VB[5])) + VC[3] * VB[4] + VC[4] * VB[3] + (-(VC[5] * VB[2])) + VC[6] * VB[1]) * VA[5];
//#pragma gpc multivector_component combined_motor e0^e3 combined_motor[3]
combined_motor[3] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[3] + (VC[0] * VB[1] + VC[1] * VB[0] + (-(VC[2] * VB[4])) + (-(VC[3] * VB[5])) + VC[4] * VB[2] + VC[5] * VB[3] + (-(VC[6] * VB[7]))) * VA[5] + (VC[0] * VB[2] + VC[1] * VB[4] + VC[2] * VB[0] + (-(VC[3] * VB[6])) + (-(VC[4] * VB[1])) + VC[5] * VB[7] + VC[6] * VB[3]) * VA[6] + (VC[0] * VB[3] + VC[1] * VB[5] + VC[2] * VB[6] + VC[3] * VB[0] + (-(VC[4] * VB[7])) + (-(VC[5] * VB[1])) + (-(VC[6] * VB[2]))) * VA[0] + (-((VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[7])) + (-((VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[1])) + (-((VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[2])) + (-((VC[0] * VB[7] + VC[1] * VB[6] + (-(VC[2] * VB[5])) + VC[3] * VB[4] + VC[4] * VB[3] + (-(VC[5] * VB[2])) + VC[6] * VB[1]) * VA[4]));
//#pragma gpc multivector_component combined_motor e1^e2 combined_motor[4]
combined_motor[4] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[4] + (VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[0] + (-((VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[6])) + (VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[5];
//#pragma gpc multivector_component combined_motor e1^e3 combined_motor[5]
combined_motor[5] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[5] + (VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[6] + (VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[0] + (-((VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[4]));
//#pragma gpc multivector_component combined_motor e2^e3 combined_motor[6]
combined_motor[6] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[6] + (-((VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[5])) + (VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[4] + (VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[0];
//#pragma gpc multivector_component combined_motor e0^e1^e2^e3 combined_motor[7]
combined_motor[7] = (VC[0] * VB[0] + (-(VC[4] * VB[4])) + (-(VC[5] * VB[5])) + (-(VC[6] * VB[6]))) * VA[7] + (VC[0] * VB[1] + VC[1] * VB[0] + (-(VC[2] * VB[4])) + (-(VC[3] * VB[5])) + VC[4] * VB[2] + VC[5] * VB[3] + (-(VC[6] * VB[7]))) * VA[6] + (-((VC[0] * VB[2] + VC[1] * VB[4] + VC[2] * VB[0] + (-(VC[3] * VB[6])) + (-(VC[4] * VB[1])) + VC[5] * VB[7] + VC[6] * VB[3]) * VA[5])) + (VC[0] * VB[3] + VC[1] * VB[5] + VC[2] * VB[6] + VC[3] * VB[0] + (-(VC[4] * VB[7])) + (-(VC[5] * VB[1])) + (-(VC[6] * VB[2]))) * VA[4] + (VC[0] * VB[4] + VC[4] * VB[0] + (-(VC[5] * VB[6])) + VC[6] * VB[5]) * VA[3] + (-((VC[0] * VB[5] + VC[4] * VB[6] + VC[5] * VB[0] + (-(VC[6] * VB[4]))) * VA[2])) + (VC[0] * VB[6] + (-(VC[4] * VB[5])) + VC[5] * VB[4] + VC[6] * VB[0]) * VA[1] + (VC[0] * VB[7] + VC[1] * VB[6] + (-(VC[2] * VB[5])) + VC[3] * VB[4] + VC[4] * VB[3] + (-(VC[5] * VB[2])) + VC[6] * VB[1]) * VA[0];
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

#line 164 "D:/Development/GAAlign/src/optimization/gradient_descent_cuda/gradient_descent_cuda.cug"


calculatedMotor[0] = out_motor[0];
calculatedMotor[1] = out_motor[1];
calculatedMotor[2] = out_motor[2];
calculatedMotor[3] = out_motor[3];
calculatedMotor[4] = out_motor[4];
calculatedMotor[5] = out_motor[5];
calculatedMotor[6] = out_motor[6];
calculatedMotor[7] = out_motor[7];



#line 169 "D:/Development/GAAlign/src/optimization/gradient_descent_cuda/gradient_descent_cuda.cug"

    // Set the calculated motor to the out motors
    for(int i=0; i<8; i++) {
        outMotors[8*idx + i] = calculatedMotor[i];
    }
}

/*
 * Sum up all motors in parallel. Implementation based on https://sodocumentation.net/cuda/topic/6566/parallel-reduction--e-g--how-to-sum-an-array-
 */
__global__ void sumMotors(const float *calculatedMotors, float* summedMotor, int motorsCount, float stepSize) {
    // 2D Thread ID
    int idx = threadIdx.x;

    // Sum up locally first, if there are more motors than the block size
    float localSum[8] = {0};
    for (int i = idx; i < motorsCount; i += blockDim.x) {
        #pragma unroll
        for(int j=0; j<8; j++) {
            localSum[j] += calculatedMotors[8*i + j];
        }
    }

    // Initialize a shared array for summing
    extern __shared__ float partialSum[];

    // Apply the current value if it is not nan
    if(!isnan(localSum[0])) {
        #pragma unroll
        for(int i=0; i<8; i++) {
            partialSum[8*idx+i] = localSum[i];
        }
    }

    // Wait until all local calculations are done
    __syncthreads();

    // Do the graph based sum (adapted from the PMPP lecture slides "2021-10-26-CUDAProgramming3" slide 55)
    unsigned int t = threadIdx.x;
    unsigned int stride;
    for (stride = blockDim.x; stride > 1;) {
        __syncthreads();
        stride = stride >> 1;
        if (t < stride) {
            #pragma unroll
            for(int j=0; j<8; j++) {
                partialSum[8*t+j] += partialSum[8*(t + stride)+j];
            }
        }
    }


    // Normalize & Scale the summed motor and then join it with the current combined motor (stored in constant memory)
    __syncthreads();

    // Normalize and scale with step size
    if(idx > 0 && idx < 8) {
        partialSum[idx] = stepSize*(partialSum[idx] / partialSum[0]);
    }

    __syncthreads();

    if(idx == 0) {
        // Print if wished
        //printf("Sum: [%f,%f,%f,%f,%f,%f,%f,%f]\n", partialSum[0], partialSum[1], partialSum[2], partialSum[3], partialSum[4], partialSum[5], partialSum[6], partialSum[7]);

        // Set the scalar component to 1
        partialSum[0] = 1;

        // Join with input motor
        // Init output
        float joinedMotor[8] = {0.0};



#line 248 "D:/Development/GAAlign/src/optimization/gradient_descent_cuda/gradient_descent_cuda.cug"
#include <math.h>
//#pragma gpc multivector combined
float combined[8];
//#pragma gpc multivector combined_norm
float combined_norm;
//#pragma gpc multivector normed_motor
float normed_motor[8];

//#pragma gpc multivector_component combined 1.0 combined[0]
combined[0] = partialSum[0] * constantMotor[0] + (-(partialSum[4] * constantMotor[4])) + (-(partialSum[5] * constantMotor[5])) + (-(partialSum[6] * constantMotor[6]));
//#pragma gpc multivector_component combined e0^e1 combined[1]
combined[1] = partialSum[0] * constantMotor[1] + partialSum[1] * constantMotor[0] + (-(partialSum[2] * constantMotor[4])) + (-(partialSum[3] * constantMotor[5])) + partialSum[4] * constantMotor[2] + partialSum[5] * constantMotor[3] + (-(partialSum[6] * constantMotor[7])) + (-(partialSum[7] * constantMotor[6]));
//#pragma gpc multivector_component combined e0^e2 combined[2]
combined[2] = partialSum[0] * constantMotor[2] + partialSum[1] * constantMotor[4] + partialSum[2] * constantMotor[0] + (-(partialSum[3] * constantMotor[6])) + (-(partialSum[4] * constantMotor[1])) + partialSum[5] * constantMotor[7] + partialSum[6] * constantMotor[3] + partialSum[7] * constantMotor[5];
//#pragma gpc multivector_component combined e0^e3 combined[3]
combined[3] = partialSum[0] * constantMotor[3] + partialSum[1] * constantMotor[5] + partialSum[2] * constantMotor[6] + partialSum[3] * constantMotor[0] + (-(partialSum[4] * constantMotor[7])) + (-(partialSum[5] * constantMotor[1])) + (-(partialSum[6] * constantMotor[2])) + (-(partialSum[7] * constantMotor[4]));
//#pragma gpc multivector_component combined e1^e2 combined[4]
combined[4] = partialSum[0] * constantMotor[4] + partialSum[4] * constantMotor[0] + (-(partialSum[5] * constantMotor[6])) + partialSum[6] * constantMotor[5];
//#pragma gpc multivector_component combined e1^e3 combined[5]
combined[5] = partialSum[0] * constantMotor[5] + partialSum[4] * constantMotor[6] + partialSum[5] * constantMotor[0] + (-(partialSum[6] * constantMotor[4]));
//#pragma gpc multivector_component combined e2^e3 combined[6]
combined[6] = partialSum[0] * constantMotor[6] + (-(partialSum[4] * constantMotor[5])) + partialSum[5] * constantMotor[4] + partialSum[6] * constantMotor[0];
//#pragma gpc multivector_component combined e0^e1^e2^e3 combined[7]
combined[7] = partialSum[0] * constantMotor[7] + partialSum[1] * constantMotor[6] + (-(partialSum[2] * constantMotor[5])) + partialSum[3] * constantMotor[4] + partialSum[4] * constantMotor[3] + (-(partialSum[5] * constantMotor[2])) + partialSum[6] * constantMotor[1] + partialSum[7] * constantMotor[0];
//#pragma gpc multivector_component combined_norm 1.0 combined_norm
combined_norm = sqrtf(fabs(combined[0] * combined[0] + (-(combined[4] * (-combined[4]))) + (-(combined[5] * (-combined[5]))) + (-(combined[6] * (-combined[6])))));
//#pragma gpc multivector_component normed_motor 1.0 normed_motor[0]
normed_motor[0] = combined[0] * combined_norm / (combined_norm * combined_norm);
//#pragma gpc multivector_component normed_motor e0^e1 normed_motor[1]
normed_motor[1] = combined[1] * combined_norm / (combined_norm * combined_norm);
//#pragma gpc multivector_component normed_motor e0^e2 normed_motor[2]
normed_motor[2] = combined[2] * combined_norm / (combined_norm * combined_norm);
//#pragma gpc multivector_component normed_motor e0^e3 normed_motor[3]
normed_motor[3] = combined[3] * combined_norm / (combined_norm * combined_norm);
//#pragma gpc multivector_component normed_motor e1^e2 normed_motor[4]
normed_motor[4] = combined[4] * combined_norm / (combined_norm * combined_norm);
//#pragma gpc multivector_component normed_motor e1^e3 normed_motor[5]
normed_motor[5] = combined[5] * combined_norm / (combined_norm * combined_norm);
//#pragma gpc multivector_component normed_motor e2^e3 normed_motor[6]
normed_motor[6] = combined[6] * combined_norm / (combined_norm * combined_norm);
//#pragma gpc multivector_component normed_motor e0^e1^e2^e3 normed_motor[7]
normed_motor[7] = combined[7] * combined_norm / (combined_norm * combined_norm);

#line 257 "D:/Development/GAAlign/src/optimization/gradient_descent_cuda/gradient_descent_cuda.cug"


joinedMotor[0] = normed_motor[0];
joinedMotor[1] = normed_motor[1];
joinedMotor[2] = normed_motor[2];
joinedMotor[3] = normed_motor[3];
joinedMotor[4] = normed_motor[4];
joinedMotor[5] = normed_motor[5];
joinedMotor[6] = normed_motor[6];
joinedMotor[7] = normed_motor[7];


#line 261 "D:/Development/GAAlign/src/optimization/gradient_descent_cuda/gradient_descent_cuda.cug"

        //printf("joinedMotor: [%f,%f,%f,%f,%f,%f,%f,%f]\n", joinedMotor[0], joinedMotor[1], joinedMotor[2], joinedMotor[3], joinedMotor[4], joinedMotor[5], joinedMotor[6], joinedMotor[7]);

        // Apply to output -> this is the slowest part!
        #pragma unroll
        for(int i=0; i<8; i++) {
            summedMotor[i] = joinedMotor[i];
        }
    }

}

gaalign::Motor gaalign::GradientDescentOptimizerCUDA::optimize(const std::vector<Correspondence> &correspondences) const {
    // Sanity checks
    if(!m_initialized) {
        std::cout << "ERROR: GradientDescentOptimizerCUDA needs to be initialized using init() before use!";
    }

    // If there are more correspondences than we need -> Randomly sample the chosen amount
    std::vector<Correspondence> sampledCorrespondences;
    sampledCorrespondences.reserve(m_settings.maxCorrespondences);
    if(correspondences.size() > m_settings.maxCorrespondences) {
        // Initialize an array containing the indices of all correspondences
        std::vector<std::uint32_t> indices(correspondences.size());
        std::iota(indices.begin(), indices.end(), 0); // Fill with values [0, 1, 2, ..]

        // Shuffle the array
        shuffle_pcg_divisionless_with_slight_bias(indices.data(), indices.size());

        // Use the first n indices
        for(int i=0; i<m_settings.maxCorrespondences; i++) {
            sampledCorrespondences.push_back(correspondences[indices[i]]);
        }
    }
    else {
        sampledCorrespondences = correspondences;
    }

    if(m_settings.verbose) std::cout << "Running CUDA Gradient descent.." << std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Put the correspondences into a large array
    #pragma omp parallel for
    for (int i = 0; i < sampledCorrespondences.size(); i++) {
        correspondenceArrayCPU[6 * i] =     (float)sampledCorrespondences[i].first.x();
        correspondenceArrayCPU[6 * i + 1] = (float)sampledCorrespondences[i].first.y();
        correspondenceArrayCPU[6 * i + 2] = (float)sampledCorrespondences[i].first.z();
        correspondenceArrayCPU[6 * i + 3] = (float)sampledCorrespondences[i].second.x();
        correspondenceArrayCPU[6 * i + 4] = (float)sampledCorrespondences[i].second.y();
        correspondenceArrayCPU[6 * i + 5] = (float)sampledCorrespondences[i].second.z();
    }

    // Upload the correspondences to the device
    cudaMemcpy(correspondencesGPU, (float*)correspondenceArrayCPU, 6 * sizeof(float) * sampledCorrespondences.size(), cudaMemcpyHostToDevice); CUDA_CHECK_ERROR;

    // Sync the devices
    cudaDeviceSynchronize();

    // Measure only upload
    auto endUpload = std::chrono::high_resolution_clock::now();
    double timeMSUpload = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(endUpload - begin).count())/1000000.0;

    // Upload an identity motor to the current constant motor
    gaalign::Motor identity = gaalign::Motor::identity();
    float* identityMotorFloat = (float*) malloc(8 * sizeof(float));
    for(int i=0; i<8; i++) {
        identityMotorFloat[i] = (float)identity.data[i];
    }
    cudaMemcpyToSymbol(constantMotor, (float*)identityMotorFloat, 8 * sizeof(float), 0, cudaMemcpyHostToDevice); CUDA_CHECK_ERROR;

    // Do the actual iterations
    for (int iter = 0; iter < m_settings.maxIterations; iter++) {
        // Execute the main kernel that computes the motors
        calculateMotors<<<1, 512, 6*sampledCorrespondences.size()*sizeof(float)>>>(correspondencesGPU, indicesGPU, calculatedMotors, iter, m_settings.trianglesPerIteration, sampledCorrespondences.size()); CUDA_CHECK_ERROR;

        // Execute a reduction kernel that calculates the averaged motor
        int reductionBlockSize = 64; // MUST be a power of two and smaller than the number of motors
        double stepSize = m_settings.stepSize;
        if(iter == 0) stepSize = 1;
        sumMotors<<<1, reductionBlockSize, /*shared memory size*/ 8*reductionBlockSize*sizeof(unsigned int)>>>(calculatedMotors, avgMotor, m_settings.trianglesPerIteration, stepSize); CUDA_CHECK_ERROR;

        // Update the motor in constant memory to the current combined motor
        cudaMemcpyToSymbol(constantMotor, (float*)avgMotor, 8 * sizeof(float), 0, cudaMemcpyDeviceToDevice); CUDA_CHECK_ERROR;
    }

    // Download the final motor and convert it to a Motor struct
    float* resultMotorArr = (float*)malloc(8 * sizeof(float));
    cudaMemcpy(resultMotorArr, (float*)avgMotor, 8 * sizeof(float), cudaMemcpyDeviceToHost); CUDA_CHECK_ERROR;

    // Unpack to motor
    Motor result;
    for(int i=0; i<8; i++) {
        result.data[i] = (double)resultMotorArr[i];
    }

    result.print();

    // End timer
    auto end = std::chrono::high_resolution_clock::now();
    double timeMS = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/1000000.0;
    if(m_settings.printTiming) std::cout << "Finished Optimization in " << timeMS << " ms (Upload took " << timeMSUpload << " ms)" << std::endl;



    return result;
}

std::string gaalign::GradientDescentOptimizerCUDA::getName() const {
    return "Gradient Descent CUDA";
}

gaalign::GradientDescentSettingsCUDA &gaalign::GradientDescentOptimizerCUDA::getSettings() {
    return m_settings;
}

void gaalign::GradientDescentOptimizerCUDA::init() {
    if(m_initialized) {
        return;
    }

    std::cout << "Initializing memory for the cuda optimizer.." << std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Allocate memory for correspondences
    cudaMalloc(&correspondencesGPU, 6 * sizeof(float) * m_settings.maxCorrespondences);

    // Allocate the memory for indices
    cudaMalloc(&indicesGPU, sizeof(unsigned int) * m_settings.trianglesPerIteration*m_settings.maxIterations);

    // Allocate the memory for the motors calculated per iteration
    cudaMalloc(&calculatedMotors, sizeof(float) * 8 * m_settings.trianglesPerIteration);

    // Allocate the memory for the averaged motor generated by the kernel
    cudaMalloc(&avgMotor, sizeof(float) * 8);

    // Also initialize the CPU memory
    correspondenceArrayCPU = (float *) malloc(6 * sizeof(float) * m_settings.maxCorrespondences);

    // End timer
    auto end = std::chrono::high_resolution_clock::now();
    double timeMS = ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count())/1000000.0;
    if(m_settings.printTiming) std::cout << "Allocated memory in " << timeMS << " ms" << std::endl;

    // Store that this optimizer was initialized
    m_initialized = true;
}

gaalign::GradientDescentOptimizerCUDA::~GradientDescentOptimizerCUDA() {
    // Free gpu memory
    cudaFree(correspondencesGPU);
    cudaFree(indicesGPU);
    cudaFree(calculatedMotors);
    cudaFree(avgMotor);

    // Also free CPU memory
    free(correspondenceArrayCPU);
}
