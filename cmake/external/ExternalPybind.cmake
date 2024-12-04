FetchContent_Declare_Fallback(
        pybind11

        URL            https://github.com/pybind/pybind11/archive/refs/tags/v2.12.0.tar.gz
        URL_HASH       SHA256=bf8f242abd1abcd375d516a7067490fb71abd79519a282d22b6e4d19282185a7

        FIND_PACKAGE_ARGS 2.12.0
)
FetchContent_MakeAvailable(pybind11)
