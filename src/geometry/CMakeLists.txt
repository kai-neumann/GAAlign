# Find the GAALOP precompiler and overwrite a few options
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
set(GPC_ALGEBRA_NAME "3dpga")
set(GPC_USE_DOUBLE ON)
FIND_PACKAGE(GPC)

# Define the library
GPC_CXX_ADD_LIBRARY(gaalign-geometry motor.h point_cloud.h motor_estimation.h motor_estimation_sse.h motor.cpg point_cloud.cpp motor_estimation.cpg motor_estimation_sse.cpp common.h)

# Set some target specific compile flags
set_target_properties(gaalign-geometry PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(gaalign-geometry PRIVATE ${OpenMP_CXX_FLAGS})

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(gaalign-geometry PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find external required libraries
find_package(Eigen3 CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED COMPONENTS common filters)

# Define the dependencies
target_link_libraries(gaalign-geometry
        PUBLIC Eigen3::Eigen pcl_common
        PRIVATE pcl_filters)
