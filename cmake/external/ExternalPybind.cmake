if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    FetchContent_Declare(
            pybind11

            URL            https://github.com/pybind/pybind11/archive/refs/tags/v2.11.1.tar.gz
            URL_HASH       SHA256=d475978da0cdc2d43b73f30910786759d593a9d8ee05b1b6846d1eb16c6d2e0c

            FIND_PACKAGE_ARGS 2.11.1
    )
else()
    FetchContent_Declare(
            pybind11

            URL            https://github.com/pybind/pybind11/archive/refs/tags/v2.11.1.tar.gz
            URL_HASH       SHA256=d475978da0cdc2d43b73f30910786759d593a9d8ee05b1b6846d1eb16c6d2e0c
    )
endif()
FetchContent_MakeAvailable(pybind11)
