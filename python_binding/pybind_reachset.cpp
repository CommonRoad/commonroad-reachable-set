#include "pybind.hpp"

namespace py = pybind11;
using namespace reach;

void export_reach(py::module& m) {
    export_reachable_set_interface(m);
    export_reachable_set(m);
}

void export_reachable_set_interface(py::module& m) {
    py::class_<ReachableSetInterface, shared_ptr<ReachableSetInterface>>(m, "ReachableSetInterface")
            .def(py::init<ConfigurationPtr const&, CollisionCheckerPtr const&>(),
                 py::arg("configuration"),
                 py::arg("collision_checker"))
            .def_readonly("config", &ReachableSetInterface::config)
            .def_readonly("time_step_start", &ReachableSetInterface::time_step_start)
            .def_readonly("time_step_end", &ReachableSetInterface::time_step_end)
            .def("compute_reachable_sets", &ReachableSetInterface::compute_reachable_sets,
                 py::arg("time_step_start") = 1, py::arg("time_step_end") = 0)
            .def("drivable_area", &ReachableSetInterface::drivable_area)
            .def("reachable_set", &ReachableSetInterface::reachable_set)
            .def("drivable_area_at_time_step",
                 &ReachableSetInterface::drivable_area_at_time_step, py::arg("time_step"))
            .def("reachable_set_at_time_step",
                 &ReachableSetInterface::reachable_set_at_time_step, py::arg("time_step"));
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
            .def("compute_reachable_sets", &ReachableSet::compute_reachable_sets)
            .def("drivable_area_at_time_step", &ReachableSet::drivable_area_at_time_step)
            .def("reachable_set_at_time_step", &ReachableSet::reachable_set_at_time_step)
            .def_readonly("map_time_to_drivable_area", &ReachableSet::map_time_to_drivable_area)
            .def_readonly("map_time_to_reachable_set", &ReachableSet::map_time_to_reachable_set);
}