# Third-party dependencies

# Header-only library Eigen
add_library(eigen INTERFACE)
target_compile_definitions(eigen INTERFACE EIGEN_DONT_ALIGN_STATICALLY)
target_include_directories(eigen INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)


