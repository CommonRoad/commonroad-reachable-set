include(FetchContent)

# Disable pybind dependency of curvilinear coordinate system
# TODO probably not required anymore with new CMake structure of CR CLCS
#set(ADD_PYTHON_BINDINGS OFF)
#set(BUILD_PYBIND11 OFF)
set(CR_CLCS_BUILD_S11N OFF)

# Option to use locally installed source directory of CR-CLCS
# Use only for development purposes
# Root directory of local CR-CLCS should be at the same level as root directory of commonroad-reachable-set
option(CRREACH_LOCAL_CLCS "Use local version of Curvilinear Coordinate System" ON)

mark_as_advanced(
        CRREACH_LOCAL_CLCS
)

if (CRREACH_LOCAL_CLCS)
    message(STATUS "Using local source directory of CR-CLCS")

    FetchContent_Declare(
            crclcs
            SOURCE_DIR "${CMAKE_SOURCE_DIR}/../commonroad-clcs/"
    )

else()
    message(STATUS "Using GitHub source directory of CR-CLCS")

    FetchContent_Declare(
            crclcs
            # Release tag v2025.1.0
            GIT_REPOSITORY  https://github.com/CommonRoad/commonroad-clcs.git
            GIT_TAG        1141fd96d93c2da0dc099ddcb03595481d1cfaf8
    )

endif()

FetchContent_MakeAvailable(crclcs)

set_property(DIRECTORY ${crclcs_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

# TODO check if this is required
mark_as_advanced(
        CR_CLCS_BUILD_S11N
)
