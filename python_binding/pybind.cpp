#include "pybind.hpp"

#include <nanobind/stl/string.h>

#include <optional>

namespace nb = nanobind;

namespace {
    std::optional<nb::module_> try_import(const char *name) {
        try {
            return nb::module_::import_(name);
        } catch (nb::python_error &e) {
            if (e.matches(nb::builtins()["ModuleNotFoundError"])) {
                const auto py_warn = nb::module_::import_("warnings");
                py_warn.attr("warn")("The module '" + std::string(name) + "' is not found. Some signatures might be incorrect and some features may not work as expected.");
                return std::nullopt;
            }
            throw;
        }
    }
}

NB_MODULE(pycrreach, m) {
    m.doc() = "Pybind module for reachable set.";
    // Import the Python bindings of the collision checker and CLCS to ensure that the necessary types are bound
    try_import("commonroad_dc.pycrcc");
    try_import("commonroad_clcs.pycrccosy");

    export_data_structures(m);
    export_utility(m);
    export_reach(m);
}
