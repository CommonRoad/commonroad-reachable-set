# ========= minimum settings
cmake_minimum_required(VERSION 3.15) # minimum required version of CMake
project(reach) # project name

option(ADD_TESTS "unit tests" OFF)


if(NOT DEFINED CRDC_DIR) # path to drivability checker
    message(SEND_ERROR "Path to drivability checker (CRDC_DIR) is not set!")
endif()


if(NOT DEFINED PYTHON_VERSION)
    message(SEND_ERROR "Please define the python version via -DPYTHON_VERSION=")
endif()

# path to vendored FCL installation in drivability checker
set(fcl_DIR ${CRDC_DIR}/build/temp.linux-x86_64-${PYTHON_VERSION}/dist/lib/cmake/fcl)

# path to vendored CCD installation in drivability checker
set(ccd_DIR ${CRDC_DIR}/build/temp.linux-x86_64-${PYTHON_VERSION}/dist/lib/ccd)


# ========= find necessary packages
find_package(Eigen3 REQUIRED) # Eigen3
find_package(ccd REQUIRED) # for collision checks
find_package(fcl REQUIRED) # for collision checks
find_library(S11N_LIBRARY s11n HINTS ${CRDC_DIR}/build/lib.linux-x86_64-${PYTHON_VERSION}/commonroad_dc) # for object serialization in C++
find_package(yaml-cpp REQUIRED) # for reading yaml files
if(ADD_TESTS)
    find_package(doctest REQUIRED) # testing with doctest
endif()
find_package(OpenMP REQUIRED) # for parallel computation


# ========= general settings
set(CMAKE_CXX_STANDARD 17) # c++ standard
set(CMAKE_CXX_STANDARD_REQUIRED ON) # c++ standard
set(CMAKE_CXX_EXTENSIONS OFF) # turn off compiler extensions

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}") # enable parallel computation
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") # enable parallel computation
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}") # enable parallel computation

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0") # for profiling
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0") # for profiling

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../) # output directory of built libraries


# ========= add sub directories
add_subdirectory(external/pybind11) # pybind header

add_subdirectory(cpp) # reachable set
add_subdirectory(python_binding)  # python binding for reachable set
