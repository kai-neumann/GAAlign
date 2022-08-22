# First define the library
add_library(gaalign-registration registration_step.h registration_step.cpp registration_pipeline.h registration_pipeline.cpp)

# Set c++ standard
set_target_properties(gaalign-registration PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(gaalign-registration PRIVATE ${OpenMP_CXX_FLAGS})

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(gaalign-registration PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Find external required libraries
find_package(Eigen3 CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED COMPONENTS common filters)

# Define the dependencies
target_link_libraries(gaalign-registration
        PUBLIC Eigen3::Eigen pcl_filters)

if(ENABLE_VISUALIZATION)
    target_link_libraries(gaalign-registration PUBLIC gaalign-visualization)
endif()