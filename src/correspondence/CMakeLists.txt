# First define the library
add_library(gaalign-correspondence)

# Set some target specific compile flags
set_target_properties(gaalign-correspondence PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(gaalign-correspondence PRIVATE ${OpenMP_CXX_FLAGS})

# Specify private and public sources
target_sources(gaalign-correspondence PUBLIC correspondence_search.h distance/distance_search.h feature/feature_search.h naive/naive_search.h
        PRIVATE distance/distance_search.cpp feature/feature_search.cpp naive/naive_search.cpp
        teaser/geometry.h teaser/fpfh.h teaser/fpfh.cc teaser/matcher.h teaser/matcher.cc) # TODO: Remove ransac.h/cpp and replace with PCL functionality

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(gaalign-correspondence PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find external required libraries
find_package(Eigen3 CONFIG REQUIRED)
#find_package(PCL CONFIG REQUIRED COMPONENTS common filters features keypoints recognition)

# Define the dependencies
target_link_libraries(gaalign-correspondence
        PUBLIC Eigen3::Eigen gaalign-geometry
        PRIVATE ${PCL_LIBRARIES})