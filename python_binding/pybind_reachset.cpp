#include "pybind.hpp"

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nanobind::literals;
using namespace reach;

void export_reach(nb::module_& m) {
    export_reachable_set(m);
}

void export_reachable_set(nb::module_& m) {
    nb::class_<ReachableSet>(m, "ReachableSet")
            .def(nb::init<ConfigurationPtr const&>(),
                "configuration"_a)
            .def(nb::init<ConfigurationPtr const&, CollisionCheckerPtr const&>(),
                "configuration"_a,
                "collision_checker"_a)
            .def_ro("config", &ReachableSet::config)
            .def_ro("collision_checker", &ReachableSet::collision_checker)
            .def_ro("step_start", &ReachableSet::step_start)
            .def_ro("step_end", &ReachableSet::step_end)
            .def("compute", &ReachableSet::compute)
            .def("drivable_area_at_step", &ReachableSet::drivable_area_at_step)
            .def("reachable_set_at_step", &ReachableSet::reachable_set_at_step)
            .def("drivable_area", &ReachableSet::drivable_area)
            .def("reachable_set", &ReachableSet::reachable_set)
            .def("prune_nodes_not_reaching_final_step", &ReachableSet::prune_nodes_not_reaching_final_step)
            .def_ro("map_step_to_drivable_area", &ReachableSet::map_step_to_drivable_area)
            .def_ro("map_step_to_reachable_set", &ReachableSet::map_step_to_reachable_set);
}