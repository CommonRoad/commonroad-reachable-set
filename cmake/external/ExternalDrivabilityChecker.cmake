include(FetchContent)

# we explicitly set build s11n to ON since it is deactivated by default in the CMake file of the drivability checker
# if it is not top level
set(CR_DC_BUILD_S11N ON)

# Option to use locally installed source directory of CRDC
# Use only for development purposes
# Root directory of local CRDC should be at the same level as root directory of commonroad-reachable-set
option(CR_REACH_LOCAL_CRDC "Use local version of Drivability Checker" OFF)

mark_as_advanced(
        CR_REACH_LOCAL_CRDC
)

if (CR_REACH_LOCAL_CRDC)
    message(STATUS "Using local source directory of CRDC")

    FetchContent_Declare(
            CommonRoadDC
            SYSTEM
            SOURCE_DIR "${CMAKE_SOURCE_DIR}/../commonroad-drivability-checker/"
    )

else ()
    message(STATUS "Using GitHub source directory of CRDC")

    FetchContent_Declare(
            CommonRoadDC
            SYSTEM
            # Release tag v2025.3.0
            GIT_REPOSITORY https://github.com/CommonRoad/commonroad-drivability-checker.git
            GIT_TAG 54e33feac3b5c377dd7d5192901530ea88848fbc
    )

endif ()

FetchContent_MakeAvailable(CommonRoadDC)

set_property(DIRECTORY ${CommonRoadDC_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
