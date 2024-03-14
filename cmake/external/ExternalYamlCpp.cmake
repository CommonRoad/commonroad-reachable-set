include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
    yaml-cpp

    SYSTEM

    # NOTE - URL download should be preferred:
    # HTTP downloads are faster than Git clones and therefore reduce configuration time

    # Git version for reference
    #GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    #GIT_TAG        yaml-cpp-0.7.0
    # or via commit:
    #GIT_TAG        4ae4cb7309c07b2d8623d7e4f01efa6321441366

    URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.7.0.tar.gz
    URL_HASH SHA256=43e6a9fcb146ad871515f0d0873947e5d497a1c9c60c58cb102a97b47208b7c3

    FIND_PACKAGE_ARGS 0.6.0
)

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# set(YAML_CPP_BUILD_TOOLS OFF)
set(YAML_BUILD_SHARED_LIBS OFF)
set(YAML_CPP_BUILD_CONTRIB OFF)
set(YAML_CPP_INSTALL OFF)
set(YAML_CPP_FORMAT_SOURCE OFF)
# set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(yaml-cpp)


# yaml-cpp::yaml-cpp is only present in some configurations, so add it if it is not already present
if(NOT TARGET yaml-cpp::yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()
