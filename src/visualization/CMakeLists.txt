# First define the library
add_library(gaalign-visualization visualize.h visualize.cpp)

# Set c++ standard
set_target_properties(gaalign-visualization PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(gaalign-visualization PRIVATE ${OpenMP_CXX_FLAGS})

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(gaalign-visualization PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find external required libraries
find_package(Eigen3 CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED COMPONENTS common visualization)

# Define the dependencies
target_link_libraries(gaalign-visualization
        PRIVATE gaalign-geometry Eigen3::Eigen ${PCL_LIBRARIES})