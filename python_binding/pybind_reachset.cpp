#include "pybind.hpp"

namespace py = pybind11;
using namespace reach;

void export_reachable_set(py::module& m) {
    export_reachability_analysis(m);
    export_reachable_set_interface(m);
}

void export_reachable_set_interface(py::module& m) {
    py::class_<ReachableSetInterface, shared_ptr<ReachableSetInterface>>(m, "ReachableSetInterface")
            .def(py::init<ConfigurationPtr const&, CollisionCheckerPtr const&>(),
                 py::arg("configuration"),
                 py::arg("collision_checker"))
            .def_readonly("reachable_set", &ReachableSetInterface::reachable_set)
            .def_readonly("time_step_start", &ReachableSetInterface::time_step_start)
            .def_readonly("time_step_end", &ReachableSetInterface::time_step_end)
            .def_readonly("map_time_to_drivable_area", &ReachableSetInterface::map_time_to_drivable_area)
            .def_readonly("map_time_to_reachable_set", &ReachableSetInterface::map_time_to_reachable_set)
            .def_readwrite("map_time_to_reachable_set_pruned",
                           &ReachableSetInterface::map_time_to_reachable_set_pruned)
            .def("config", &ReachableSetInterface::config)
            .def("compute_reachable_sets", &ReachableSetInterface::compute_reachable_sets,
                 py::arg("time_step_start") = 1,py::arg("time_step_end") = 0)
             .def("compute_at_time_step", &ReachableSetInterface::compute_at_time_step,
                  py::arg("time_step"))
            .def("drivable_area_at_time_step", &ReachableSetInterface::drivable_area_at_time_step,
                 py::arg("time_step"))
            .def("reachable_set_at_time_step", &ReachableSetInterface::reachable_set_at_time_step,
                 py::arg("time_step"))
            .def("pruned_reachable_set_at_time_step", &ReachableSetInterface::pruned_reachable_set_at_time_step,
                 py::arg("time_step"));
    //.def("add_pruned_node", &ReachableSetInterface::add_pruned_node,
    //     py::arg("time_step"), py::arg("node"));
}

void export_reachability_analysis(py::module& m) {
    py::class_<ReachableSet, shared_ptr<ReachableSet>>(m, "ReachableSet")
            .def(py::init<ConfigurationPtr const&>(), py::arg("configuration"))
            .def(py::init<ConfigurationPtr const&, CollisionCheckerPtr const&>(),
                 py::arg("configuration"),
                 py::arg("collision_checker"))
            .def("config", &ReachableSet::config)
            .def("collision_checker", &ReachableSet::collision_checker)
            .def("construct_initial_drivable_area", &ReachableSet::construct_initial_drivable_area)
            .def("construct_initial_reachable_set", &ReachableSet::construct_initial_reachable_set)
            .def("compute_drivable_area_at_time_step",
                 &ReachableSet::compute_drivable_area_at_time_step)
            .def("compute_reachable_set_at_time_step",
                 &ReachableSet::compute_reachable_set_at_time_step);
}
