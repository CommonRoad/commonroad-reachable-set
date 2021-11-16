#include "pybind.hpp"

namespace py = pybind11;
using namespace reach;

void export_utility(py::module& m) {
    m.def("print_vertices_polygon", &print_vertices_polygon);
    m.def("create_curvilinear_collision_checker", &create_curvilinear_collision_checker);
    m.def("print_collision_checker", &print_collision_checker);
}