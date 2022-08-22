# First define the library
add_executable(gaalign-testbench testbench.cpp common.h synthetic_data_generator.cpp synthetic_data_generator.h algorithm_comparison.cpp algorithm_comparison.h
        wrappers/algorithm_wrapper.h wrappers/gaalign_wrapper.cpp wrappers/gaalign_wrapper.h wrappers/pcl_icp_wrapper.cpp wrappers/pcl_icp_wrapper.h
        gradient_descent_hyperparameter_optimization.cpp gradient_descent_hyperparameter_optimization.h wrappers/opencv_icp_wrapper.cpp wrappers/opencv_icp_wrapper.h
        wrappers/super4pcs_wrapper.cpp wrappers/super4pcs_wrapper.h wrappers/go_icp_wrapper.cpp wrappers/go_icp_wrapper.h wrappers/fgr_wrapper.cpp wrappers/fgr_wrapper.h
        robustness_tests.h robustness_tests.cpp wrappers_optim/optimization_wrapper.h wrappers_optim/gaalign_optimization_wrapper.cpp wrappers_optim/gaalign_optimization_wrapper.h wrappers_optim/pcl_optimization_wrapper.cpp wrappers_optim/pcl_optimization_wrapper.h wrappers_optim/gaalign_cuda_optimization_wrapper.cpp wrappers_optim/gaalign_cuda_optimization_wrapper.h performance_evaluation.cpp performance_evaluation.h common.cpp hyperparameter_evaluation.cpp hyperparameter_evaluation.h wrappers_optim/ga_lms_optimization_wrapper.cpp wrappers_optim/ga_lms_optimization_wrapper.h wrappers/ga_lms_wrapper.cpp wrappers/ga_lms_wrapper.h game/MotorEstimationSolver.h game/scoped_ptr.h wrappers_optim/game_optimization_wrapper.cpp wrappers_optim/game_optimization_wrapper.h)

# find matplotlib
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
find_package(Python3 COMPONENTS Interpreter NumPy REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Set some target specific compile flags
set_target_properties(gaalign-testbench PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(gaalign-testbench PRIVATE ${OpenMP_CXX_FLAGS})

# Find external libraries
find_package(Boost 1.36.0 COMPONENTS filesystem program_options)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)

# Define the dependencies
target_link_libraries(gaalign-testbench
        PRIVATE gaalign-geometry gaalign-registration gaalign-correspondence gaalign-optimization
        ${PYTHON_LIBRARIES} Python3::NumPy Boost::boost Boost::filesystem Boost::program_options Eigen3::Eigen ${OpenCV_LIBS} Ceres::ceres)

if(ENABLE_CUDA)
    target_link_libraries(gaalign-testbench
            PRIVATE gaalign-optimization-cuda)
endif()

if(ENABLE_VISUALIZATION)
    target_link_libraries(gaalign-testbench PRIVATE gaalign-visualization)
endif()