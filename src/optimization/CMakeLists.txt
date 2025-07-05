# First define the library
add_library(gaalign-optimization)

# Set c++ standard
set_target_properties(gaalign-optimization PROPERTIES CXX_STANDARD 17)

# Enable openmp
find_package(OpenMP REQUIRED)
target_compile_options(gaalign-optimization PRIVATE ${OpenMP_CXX_FLAGS})

# Specify private and public sources
target_sources(gaalign-optimization
        PUBLIC optimizer.h gradient_descent/gradient_descent.h
        PRIVATE gradient_descent/gradient_descent.cpp
        fast_shuffle.cpp fast_shuffle.h fast_shuffle_pcg.h)

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(gaalign-optimization PUBLIC ${CMAKE_SOURCE_DIR}/src)


# Define the dependencies
target_link_libraries(gaalign-optimization PUBLIC gaalign-geometry)


# -------------------------------------------
# Define a separate library for cuda optimization

if(ENABLE_CUDA)

    # Find the GAALOP precompiler and overwrite a few options
    add_library(gaalign-optimization-cuda gradient_descent_cuda/gradient_descent_cuda.h gradient_descent_cuda/gradient_descent_cuda.cu fast_shuffle.cpp fast_shuffle.h fast_shuffle_pcg.h)

    # Set c++ standard
    set_target_properties(gaalign-optimization-cuda PROPERTIES CXX_STANDARD 17)

    # Enable openmp
    target_compile_options(gaalign-optimization-cuda PRIVATE -Xcompiler="${OpenMP_CXX_FLAGS}")

    # Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
    target_include_directories(gaalign-optimization-cuda PUBLIC ${CMAKE_SOURCE_DIR}/src)

    # Define the dependencies
    target_link_libraries(gaalign-optimization-cuda PRIVATE gaalign-geometry)

endif()