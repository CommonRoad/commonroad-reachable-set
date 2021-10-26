#include "pybind.hpp"

namespace py = pybind11;
using namespace reach;

void export_reachable_set(py::module& m) {
    export_continuous_reachability_analysis(m);
    export_reachable_set_interface(m);
}

void export_reachable_set_interface(py::module& m) {
    py::class_<ReachableSetInterface, shared_ptr<ReachableSetInterface>>(m, "ReachableSetInterface")
            .def_readonly("time_step_start", &ReachableSetInterface::time_step_start)
            .def_readwrite("time_step_end", &ReachableSetInterface::time_step_end)
            .def_readonly("map_time_to_drivable_area", &ReachableSetInterface::map_time_to_drivable_area)
            .def_readonly("map_time_to_reachable_set", &ReachableSetInterface::map_time_to_reachable_set)
            .def_readwrite("map_time_to_reachable_set_refined",
                           &ReachableSetInterface::map_time_to_reachable_set_refined)
            .def(py::init<ContinuousReachabilityAnalysisPtr&>(),
                 py::arg("reachability_analysis"))
            .def(py::init<SemanticReachabilityAnalysisPtr&>(),
                 py::arg("reachability_analysis"))
            .def("config", &ReachableSetInterface::config)
            .def("print_collision_checker_info", &ReachableSetInterface::print_collision_checker_info)
            .def("continuous", &ReachableSetInterface::continuous)
            .def("semantic", &ReachableSetInterface::semantic)
            .def("compute", &ReachableSetInterface::compute,
                 py::arg("time_step_start") = 1,
                 py::arg("time_step_end") = 0)
            .def("drivable_area_at_time_step", &ReachableSetInterface::drivable_area_at_time_step,
                 py::arg("time_step"))
            .def("reachable_set_at_time_step", &ReachableSetInterface::reachable_set_at_time_step,
                 py::arg("time_step"))
            .def("refined_reachable_set_at_time_step", &ReachableSetInterface::refined_reachable_set_at_time_step,
                 py::arg("time_step"))
            .def("add_refined_node", &ReachableSetInterface::add_refined_node, py::arg("time_step"), py::arg("node"));
}

void export_continuous_reachability_analysis(py::module& m) {
    py::class_<ContinuousReachabilityAnalysis,
            shared_ptr<ContinuousReachabilityAnalysis>>(m, "ContinuousReachabilityAnalysis")
            .def(py::init<ConfigurationPtr const&>(), py::arg("configuration"))
            .def(py::init<ConfigurationPtr const&, CollisionCheckerPtr const&>(),
                 py::arg("configuration"),
                 py::arg("collision_checker"))
            .def("config", &ContinuousReachabilityAnalysis::config)
            .def("collision_checker", &ContinuousReachabilityAnalysis::collision_checker)
            .def("initial_drivable_area", &ContinuousReachabilityAnalysis::initial_drivable_area)
            .def("initial_reachable_set", &ContinuousReachabilityAnalysis::initial_reachable_set)
            .def("compute_drivable_area_at_time_step",
                 &ContinuousReachabilityAnalysis::compute_drivable_area_at_time_step)
            .def("compute_reachable_set_at_time_step",
                 &ContinuousReachabilityAnalysis::compute_reachable_set_at_time_step);
}
