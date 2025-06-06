set(python_module_name pycrreach)
set(install_destination commonroad_reach)
nanobind_add_module(${python_module_name} MODULE
        pybind.hpp
        pybind.cpp
        pybind_data_structure.cpp
        pybind_reachset.cpp
        pybind_utility.cpp
)
target_link_libraries(${python_module_name} PRIVATE crreach)

install(TARGETS ${python_module_name}
        LIBRARY DESTINATION ${install_destination}
        COMPONENT ${python_module_name}
)

add_custom_target(install-python-modules-${python_module_name}
        ${CMAKE_COMMAND}
        -DCMAKE_INSTALL_COMPONENT=${python_module_name}
        -P "${PROJECT_BINARY_DIR}/cmake_install.cmake"
        DEPENDS ${python_module_name}
)

target_precompile_headers(nanobind-static
        PUBLIC
        <nanobind/nanobind.h>
)

if (SKBUILD)
    # only add Python typing stubs if we are building with scikit-build, otherwise, Python dependencies might be missing
    nanobind_add_stub(
            ${python_module_name}_stub
            MODULE ${python_module_name}
            OUTPUT ${python_module_name}.pyi
            PYTHON_PATH $<TARGET_FILE_DIR:${python_module_name}>
            DEPENDS ${python_module_name}
            MARKER_FILE py.typed
            VERBOSE
    )

    install(FILES
            ${CMAKE_CURRENT_BINARY_DIR}/${python_module_name}.pyi
            ${CMAKE_CURRENT_BINARY_DIR}/py.typed
            DESTINATION ${install_destination}
            COMPONENT ${python_module_name}
    )

    # Sanity check: Ensure we are building all dependencies as static libraries
    include(utils/EnsureStatic)
    if (SKBUILD_SELF_CONTAINED)
        # If we are building all dependencies ourselves, then they should
        # all be static.
        ensure_all_static(crcc)
    else ()
        # If we are partially using system libraries, then only the crcc library
        # itself has to be static.
        ensure_static(crcc)
    endif ()
endif ()
