# create pybind target with source files
pybind11_add_module(pycrreach MODULE
        pybind.cpp
        pybind_data_structure.cpp
        pybind_utility.cpp
        pybind_reachset.cpp
        ${REACH_SOURCE})
# add include directories
target_include_directories(pycrreach PRIVATE
        ${REACH_INCLUDE_DIR})
# link to external libraries
target_link_libraries(pycrreach PRIVATE
        yaml-cpp
        ${CRDC_LIBRARY}
        fcl
        ccd
        ${S11N_LIBRARY}
        pybind11::module)

message(STATUS "Found python bindings for pycrreach.")

