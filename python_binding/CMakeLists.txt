include(utils/EnsureStatic)

# Sanity check: Ensure we are building all dependencies as static libraries
if(SKBUILD)
    if(SKBUILD_SELF_CONTAINED)
        # If we are building all dependencies ourselves, then they should
        # all be static.

        ensure_all_static(crreach)
    else()
        # If we are partially using system libraries, then only the crreach library
        # itself has to be static.

        ensure_static(crreach)
    endif()
endif()

find_package(pybind11 2.7.0 QUIET)

# add pybind module
if(SKBUILD)
    message(STATUS "The project is built using scikit-build")
endif()
if(pybind11_FOUND AND NOT SKBUILD)
    message(STATUS "Adding Python interface for compilation only")
endif()
if(pybind11_FOUND)
    add_library(crreach_python OBJECT
            pybind.hpp
            pybind.cpp
            pybind_data_structure.cpp
            pybind_utility.cpp
            pybind_reachset.cpp)
    target_link_libraries(crreach_python PRIVATE crreach pybind11::module)
    set_property(TARGET crreach_python PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

if(SKBUILD)
    set(python_module_name pycrreach)
    pybind11_add_module(${python_module_name} MODULE $<TARGET_OBJECTS:crreach_python>)

    target_link_libraries(${python_module_name} PRIVATE crreach)

    install(TARGETS ${python_module_name}
            LIBRARY DESTINATION commonroad_reach
            COMPONENT ${python_module_name}
    )

    add_custom_target(install-python-modules-${python_module_name}
            ${CMAKE_COMMAND}
            -DCMAKE_INSTALL_COMPONENT=${python_module_name}
            -P "${PROJECT_BINARY_DIR}/cmake_install.cmake"
            DEPENDS ${python_module_name}
    )

    # include(utils/CheckRuntimeDependencies)
    # check_python_runtime_dependencies(${python_module_name})
endif(SKBUILD)
