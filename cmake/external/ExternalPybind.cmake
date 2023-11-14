if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    FetchContent_Declare(
            pybind11

            URL            https://github.com/pybind/pybind11/archive/refs/tags/v2.10.4.tar.gz
            URL_HASH       SHA256=832e2f309c57da9c1e6d4542dedd34b24e4192ecb4d62f6f4866a737454c9970

            FIND_PACKAGE_ARGS 2.10.4
    )
else()
    FetchContent_Declare(
            pybind11

            URL            https://github.com/pybind/pybind11/archive/refs/tags/v2.10.4.tar.gz
            URL_HASH       SHA256=832e2f309c57da9c1e6d4542dedd34b24e4192ecb4d62f6f4866a737454c9970
    )
endif()
FetchContent_MakeAvailable(pybind11)
