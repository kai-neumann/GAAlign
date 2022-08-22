# First define the library
add_executable(gaalign-app main.cpp)

# Set some target specific compile flags
set_target_properties(gaalign-app PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(gaalign-app PRIVATE ${OpenMP_CXX_FLAGS})

# Find external libraries
find_package(Boost 1.36.0 COMPONENTS filesystem program_options)
find_package(Eigen3 CONFIG REQUIRED)

# Define the dependencies
target_link_libraries(gaalign-app
        PUBLIC gaalign-geometry gaalign-registration gaalign-correspondence gaalign-optimization Boost::boost Boost::filesystem Boost::program_options Eigen3::Eigen)

if(ENABLE_CUDA)
    target_link_libraries(gaalign-app PRIVATE gaalign-optimization-cuda)
endif()

# Add directories
target_link_directories(gaalign-app PRIVATE gaalign-geometry)

# All install commands get the same destination. this allows us to use paths
# relative to the executable.
install(TARGETS gaalign-app DESTINATION example_destination)

# Now comes everything we need, to create a package
# there are a lot more variables you can set, and some
# you need to set for some package types, but we want to
# be minimal here.
set(CPACK_PACKAGE_NAME "GAAlign")
set(CPACK_PACKAGE_VERSION "1.0.0")

# We don't want to split our program up into several incomplete pieces.
set(CPACK_MONOLITHIC_INSTALL 1)

# This must be last
include(CPack)