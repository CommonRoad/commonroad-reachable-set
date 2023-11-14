# ========= minimum settings
cmake_minimum_required(VERSION 3.20..3.26) # minimum required version of CMake
project(reach) # project name
option(ADD_TESTS "unit tests" OFF) # flag to add unit tests

# CMP0135 - URL download timestamp
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# make FetchContent available
include(FetchContent)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/external) # add path to cmake files for external packages

# ========= general settings
set(CMAKE_CXX_STANDARD 17) # c++ standard
set(CMAKE_CXX_STANDARD_REQUIRED ON) # c++ standard
set(CMAKE_CXX_EXTENSIONS OFF) # turn off compiler extensions
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../) # output directory of built libraries

# Necessary for Python modules: never build as shared library
# IMPORTANT: This must be set before finding the necessary packages
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ========= find necessary packages
find_package(yaml-cpp REQUIRED) # for reading yaml files
find_package(OpenMP REQUIRED) # for parallel computation
find_package(Eigen3 REQUIRED)
if(ADD_TESTS)
    find_package(doctest REQUIRED) # testing with doctest
endif()

include(ExternalDrivabilityChecker)

# ========= add sub directories
add_subdirectory(external/pybind11) # pybind header
add_subdirectory(cpp) # reachable set
add_subdirectory(python_binding)  # python binding for reachable set
