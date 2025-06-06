include(FetchContent)

# we explicitly set build s11n to ON since it is deactivated by default in the CMake file of the CLCS if it is not top level
set(CR_CLCS_BUILD_S11N ON)

# Option to use locally installed source directory of CR-CLCS
# Use only for development purposes
# Root directory of local CR-CLCS should be at the same level as root directory of commonroad-reachable-set
option(CR_REACH_LOCAL_CLCS "Use local version of Curvilinear Coordinate System" OFF)

mark_as_advanced(
        CR_REACH_LOCAL_CLCS
)

if (CR_REACH_LOCAL_CLCS)
    message(STATUS "Using local source directory of CR-CLCS")

    FetchContent_Declare(
            CommonRoadCLCS
            SYSTEM
            SOURCE_DIR "${CMAKE_SOURCE_DIR}/../commonroad-clcs/"
    )

else ()
    message(STATUS "Using GitHub source directory of CR-CLCS")

    FetchContent_Declare(
            CommonRoadCLCS
            SYSTEM
            # Release tag v2025.2.0
            GIT_REPOSITORY https://github.com/CommonRoad/commonroad-clcs.git
            GIT_TAG 34040496aa71a6244c99ce3236d3c395731b940b
    )

endif ()

FetchContent_MakeAvailable(CommonRoadCLCS)

set_property(DIRECTORY ${CommonRoadCLCS_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

mark_as_advanced(
        CR_CLCS_BUILD_S11N
)
