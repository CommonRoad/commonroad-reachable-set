include(FetchContent)

find_package(Threads REQUIRED)

# Required for LINK_LIBRARIES_ONLY_TARGETS (gtest links directly to pthread)
add_library(pthread ALIAS Threads::Threads)

FetchContent_Declare(
    crdc
    GIT_REPOSITORY  git@gitlab.lrz.de:cps/commonroad-drivability-checker.git
    GIT_TAG         cb14d49603578e5cf48b5c0859eeecbcba572532
    #GIT_TAG        development
)

FetchContent_MakeAvailable(crdc)

set_property(DIRECTORY ${crdc_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

mark_as_advanced(
    ADD_MODULE_GEOMETRY
    ADD_MODULE_COLLISION
    ADD_TRIANGLE
    BUILD_S11N
)
