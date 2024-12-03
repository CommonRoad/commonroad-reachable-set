include(FetchContent)

find_package(Threads REQUIRED)

# Required for LINK_LIBRARIES_ONLY_TARGETS (gtest links directly to pthread)
add_library(pthread ALIAS Threads::Threads)

# Disable pybind dependency of drivability checker
set(ADD_PYTHON_BINDINGS OFF)
set(BUILD_PYBIND11 OFF)

# Option to use locally installed source directory of CRDC
# Use only for development purposes
# Root directory of local CRDC should be at the same level as root directory of commonroad-reachable-set
option(CRREACH_LOCAL_CRDC "Use local version of Drivability Checker" OFF)

mark_as_advanced(
    CRREACH_LOCAL_CRDC
)

if (CRREACH_LOCAL_CRDC)
    message(STATUS "Using local source directory of CRDC")

    FetchContent_Declare(
        crdc
        SOURCE_DIR "${CMAKE_SOURCE_DIR}/../commonroad-drivability-checker/"
    )

else()
    message(STATUS "Using GitHub source directory of CRDC")

    FetchContent_Declare(
        crdc
        GIT_REPOSITORY  https://github.com/CommonRoad/commonroad-drivability-checker.git
        GIT_TAG        development
    )

endif()

FetchContent_MakeAvailable(crdc)

set_property(DIRECTORY ${crdc_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

mark_as_advanced(
    ADD_MODULE_GEOMETRY
    ADD_MODULE_COLLISION
    ADD_TRIANGLE
    BUILD_S11N
)
