# Provides configure_package_config_file
include(CMakePackageConfigHelpers)
# Includes sane defaults for installation paths (CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR etc.)
include(GNUInstallDirs)

set(export_name ${PROJECT_NAME}_Targets)

# Citing CMake documentation:
# For regular executables, static libraries and shared libraries,
# the DESTINATION argument is not required.
# https://cmake.org/cmake/help/v3.25/command/install.html#targets
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
    install(TARGETS crreach
            EXPORT ${export_name}
            FILE_SET crreach_headers
            LIBRARY ARCHIVE RUNTIME)
else()
    install(TARGETS crreach
            EXPORT ${export_name}
            LIBRARY ARCHIVE RUNTIME)
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/cpp/include/
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
            FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
endif()

# Define INSTALL_..._FROM_SYSTEM for each dependency depending on whether we're using
# our own version (via FetchContent) or a system version (via find_package)
# This information is used in the installed config file.
include(FetchContent)
foreach(fc_target Eigen3 yaml-cpp)
    FetchContent_GetProperties(${fc_target})

    message(DEBUG "check ${fc_target} (variable name: ${fc_target}_SOURCE_DIR)")
    if(DEFINED ${fc_target}_SOURCE_DIR)
        message(DEBUG "defined: ${${fc_target}_SOURCE_DIR}")
        set(INSTALL_${fc_target}_FROM_SYSTEM OFF)
    else()
        message(DEBUG "not defined")
        set(INSTALL_${fc_target}_FROM_SYSTEM ON)
    endif()
endforeach()

set(crreach_install_cmakedir ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

set(crreach_config_file ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake)
set(crreach_version_file ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake)

configure_package_config_file(
        ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
        ${crreach_config_file}
        INSTALL_DESTINATION ${crreach_install_cmakedir})

write_basic_package_version_file(${crreach_version_file}
        COMPATIBILITY AnyNewerVersion)

install(FILES ${crreach_config_file}
              ${crreach_version_file}
        DESTINATION  ${crreach_install_cmakedir})

# Export target configuration (for installation)
install(EXPORT ${export_name}
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${crreach_install_cmakedir}
        )

if(_enable_package_export)
    # Export target configuration (allows find_package to find local build tree without
    # first installing it)
    export(EXPORT ${export_name}
            FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
            NAMESPACE ${PROJECT_NAME}::
    )
    #elseif(TARGET eigen)
    #    # Workaround for required Eigen export
    #    export(TARGETS eigen
    #            FILE ${PROJECT_BINARY_DIR}/eigen-export-private.cmake
    #            NAMESPACE ${PROJECT_NAME}_private::
    #            )
endif()

# Required for export()
foreach(foreign_target yaml-cpp::yaml-cpp Eigen3::Eigen)
    get_target_property(aliased_target ${foreign_target} ALIASED_TARGET)
    if(aliased_target)
        set(foreign_target ${aliased_target})
    endif()

    # check if target is imported (i.e., not built locally)
    get_target_property(target_imported ${foreign_target} IMPORTED)
    # check if target is already exported (i.e., in another export set) to avoid duplicated exports
    get_target_property(target_exported ${foreign_target} EXPORT_NAME)
    if(NOT target_imported AND NOT target_exported)
        message(VERBOSE "Exporting ${foreign_target} because it is built locally")
        install(TARGETS ${foreign_target}
                EXPORT ${export_name}
                LIBRARY ARCHIVE RUNTIME)
    endif()
endforeach()
