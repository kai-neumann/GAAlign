# First define the library
add_library(gaalet INTERFACE)

# Specify private and public sources
target_sources(gaalet
        PUBLIC gaalet.h)

# Define which files are visible from outside. Use "${CMAKE_SOURCE_DIR}/src" to make the files addressable as <libfolder/file.h>
target_include_directories(gaalign-optimization PUBLIC ${CMAKE_SOURCE_DIR}/src)
