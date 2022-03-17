#include "pybind.hpp"

namespace py = pybind11;
using namespace reach;

void export_reach(py::module& m) {
    export_reachable_set(m);
}

void export_reachable_set(py::module& m) {
    py::class_<ReachableSet, shared_ptr<ReachableSet>>(m, "ReachableSet")
            .def(py::init<ConfigurationPtr const&>(),
                 py::arg("configuration"))
            .def(py::init<ConfigurationPtr const&, CollisionCheckerPtr const&>(),
                 py::arg("configuration"),
                 py::arg("collision_checker"))
            .def_readonly("config", &ReachableSet::config)
            .def_readonly("collision_checker", &ReachableSet::collision_checker)
            .def_readonly("time_step_start", &ReachableSet::time_step_start)
            .def_readonly("time_step_end", &ReachableSet::time_step_end)
            .def("compute", &ReachableSet::compute)
            .def("drivable_area_at_time_step", &ReachableSet::drivable_area_at_time_step)
            .def("reachable_set_at_time_step", &ReachableSet::reachable_set_at_time_step)
            .def("drivable_area", &ReachableSet::drivable_area)
            .def("reachable_set", &ReachableSet::reachable_set)
            .def_readonly("map_time_to_drivable_area", &ReachableSet::map_time_to_drivable_area)
            .def_readonly("map_time_to_reachable_set", &ReachableSet::map_time_to_reachable_set);
}