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

if (SKBUILD)
    message(STATUS "Installing Python bindings (pycrreach) via scikit-build-core.")
    install(TARGETS pycrreach
            LIBRARY DESTINATION ${SKBUILD_PLATLIB_DIR}/commonroad_reach)
else()
    message(STATUS "Installing Python bindings (pycrreach) locally: ${CMAKE_CURRENT_LIST_DIR}/../commonroad_reach")
    set_target_properties(pycrreach PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../commonroad_reach)
endif()
