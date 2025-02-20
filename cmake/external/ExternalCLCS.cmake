include(FetchContent)

# we explicitly set build s11n to ON since it is deactivated by default in the CMake file of the CLCS if it is not to level
set(CR_CLCS_BUILD_S11N ON)

# Option to use locally installed source directory of CR-CLCS
# Use only for development purposes
# Root directory of local CR-CLCS should be at the same level as root directory of commonroad-reachable-set
option(CRREACH_LOCAL_CLCS "Use local version of Curvilinear Coordinate System" OFF)

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
            # Release tag v2025.1.1
            GIT_REPOSITORY  https://github.com/CommonRoad/commonroad-clcs.git
            GIT_TAG        3b24b569fc0f7e887ddf2cd4bc8ca63a2495dfc0
    )

endif()

FetchContent_MakeAvailable(crclcs)

set_property(DIRECTORY ${crclcs_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

mark_as_advanced(
        CR_CLCS_BUILD_S11N
)
