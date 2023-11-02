cmake_minimum_required (VERSION 3.12)

#------------------------------------------------------------------------------
## Project setup
project (xCoreSDK VERSION 0.3.4)

message(STATUS "")
message(STATUS "    == ${PROJECT_NAME} project configuration ==")
message(STATUS "")

#------------------------------------------------------------------------------
## General settings

# Setup output path
include(GNUInstallDirs)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib" CACHE PATH "Archive output dir.")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib" CACHE PATH "Library output dir.")
set(CMAKE_PDB_OUTPUT_DIRECTORY     "${CMAKE_BINARY_DIR}/bin" CACHE PATH "PDB (MSVC debug symbol)output dir.")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin" CACHE PATH "Executable/dll output dir.")

# default build type: release
if(NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "The type of build, options are: Debug, Release, RelWithDebInfo, MinSizeRel."  FORCE )
endif()

# Compiler options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Be nice to visual studio
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#-------------------------------------------------------------------------------
## Compile options

# Using static/shared libraries option
option(XCORE_LINK_SHARED_LIBS "Example executables link shared library" OFF)
# Compile option: whether using xMateModel library. Only Linux x86_64 system and Windows release supports
option(XCORE_USE_XMATE_MODEL "xMate model library for calculation" OFF)

#-------------------------------------------------------------------------------
# Set default install location to dist folder in build dir
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/dist" CACHE PATH
        "Install path prefix, prepended onto install directories." FORCE )
endif()

#------------------------------------------------------------------------------
# Included CMakeLists.txt

# External resources/repositories are downloaded here
add_subdirectory(external)

# Public headers
add_subdirectory(include)

# xCore-SDK libraries
add_subdirectory(lib)

# Example usages
add_subdirectory(example)

# doc
add_subdirectory(doc)

#-------------------------------------------------------------------------------
# Wrap up of settings printed on build
message(STATUS "")
message(STATUS "    == Final overview for ${PROJECT_NAME} ==")
message(STATUS "Version:              ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH} ${VERSION_TYPE} @ ${VERSION_HOST}")
message(STATUS "Install prefix:       ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Compiler:             ${CMAKE_CXX_COMPILER}")
message(STATUS "CMAKE_BUILD_TYPE:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  possible options: Debug Release RelWithDebInfo MinSizeRel")
message(STATUS "  set with ` cmake -DCMAKE_BUILD_TYPE=Debug .. `")
message(STATUS "")


