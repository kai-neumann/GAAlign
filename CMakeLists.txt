cmake_minimum_required(VERSION 3.18) # setting this is required
project(gaalign CXX) # this sets the project name

# Global Project Settings
option(ENABLE_CUDA "Enables CUDA based optimizer" OFF)
option(ENABLE_TESTBENCH "Builds the testbench used for evaluation of the software" OFF)
option(ENABLE_VISUALIZATION "Enable VTK based visualization" OFF)


# Also enable CUDA
if(ENABLE_CUDA)
    add_definitions(-DENABLE_CUDA)
    enable_language(CUDA)
endif()

# Add 3rd Party libraries to prefix path
#list(APPEND CMAKE_PREFIX_PATH "./3rdparty/install")
#find_package(teaserpp REQUIRED)

# PCL needs to be included here, or else it causes linker errors
find_package(PCL CONFIG REQUIRED COMPONENTS common filters features keypoints recognition visualization)

# Enable visualization
if(ENABLE_VISUALIZATION)
    add_definitions(-DENABLE_VISUALIZATION)
    add_subdirectory(src/visualization)
endif()

# Add libraries
add_subdirectory(src/geometry)
add_subdirectory(src/correspondence)
add_subdirectory(src/optimization)
add_subdirectory(src/registration)

# Add Applications
add_subdirectory(src/app)

if(ENABLE_TESTBENCH)
    add_subdirectory(src/gaalet)
    add_subdirectory(src/testbench)
endif()

# Find the resources and copy them to the local build folders
file(GLOB_RECURSE data resources/*)
file(GLOB_RECURSE models resources/models/*)
file(GLOB_RECURSE 3rdparty 3rdparty/*)
file(COPY ${data} DESTINATION resources)
file(COPY ${3rdparty} DESTINATION 3rdparty)
file(COPY ${models} DESTINATION resources/models)
# install(DIRECTORY resources DESTINATION example_destination) # MAYBE?
