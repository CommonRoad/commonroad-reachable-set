# create pybind target with source files
pybind11_add_module(pycrreach MODULE
        pybind.cpp
        pybind_data_structure.cpp
        pybind_utility.cpp
        pybind_reachset.cpp)
# link to external libraries
target_link_libraries(pycrreach PRIVATE
        crreach
        pybind11::module)

message(STATUS "Found python bindings for pycrreach.")

