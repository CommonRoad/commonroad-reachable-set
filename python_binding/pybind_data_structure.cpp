#include "pybind.hpp"

#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nanobind::literals;
using namespace reach;

void export_data_structures(nb::module_ &m) {
    export_configuration(m);
    export_reach_polygon(m);
    export_reach_node(m);
}

void export_reach_polygon(nb::module_ &m) {
    nb::class_<ReachPolygon>(m, "ReachPolygon")
            .def(nb::init<vector<tuple<double, double>> const &>(),
                 "vec_vertices"_a)
            .def_prop_ro("p_min", &ReachPolygon::p_min)
            .def_prop_ro("p_max", &ReachPolygon::p_max)
            .def_prop_ro("v_min", &ReachPolygon::v_min)
            .def_prop_ro("v_max", &ReachPolygon::v_max)
            .def_prop_ro("p_lon_min", &ReachPolygon::p_lon_min)
            .def_prop_ro("p_lon_max", &ReachPolygon::p_lon_max)
            .def_prop_ro("p_lat_min", &ReachPolygon::p_lat_min)
            .def_prop_ro("p_lat_max", &ReachPolygon::p_lat_max)
            .def_prop_ro("p_lon_center", &ReachPolygon::p_lon_center)
            .def_prop_ro("p_lat_center", &ReachPolygon::p_lat_center)
            .def_prop_ro("bounds", &ReachPolygon::bounding_box)
            .def_prop_ro("vertices", [](ReachPolygon const &polygon) {
                nb::list list_tuples_vertices;
                for (auto const &vertex: polygon.vertices()) {
                    list_tuples_vertices.append(nb::make_tuple(vertex.p_lon(), vertex.p_lat()));
                }
                return list_tuples_vertices;
            })
            .def("convexify", &ReachPolygon::convexify)
            .def("minkowski_sum", &ReachPolygon::minkowski_sum)
            .def("intersects", &ReachPolygon::intersects)
            .def("intersect_halfspace", &ReachPolygon::intersect_halfspace, "a"_a, "b"_a, "c"_a)
            .def("__repr__", [](ReachPolygon const &polygon) {
                return "(" + std::to_string(polygon.p_lon_min()) + ", " + std::to_string(polygon.p_lat_min())
                       + ", " + std::to_string(polygon.p_lon_max()) + ", " + std::to_string(polygon.p_lat_max()) + ")";
            });
}

void export_reach_node(nb::module_ &m) {
    nb::class_<ReachNode>(m, "ReachNode")
            .def(nb::init<int const &, ReachPolygonPtr const &, ReachPolygonPtr const &>(),
                 "step"_a,
                 "polygon_lon"_a,
                 "polygon_lat"_a)
            .def_prop_ro("p_lon_min", &ReachNode::p_lon_min)
            .def_prop_ro("p_lon_max", &ReachNode::p_lon_max)
            .def_prop_ro("p_lat_min", &ReachNode::p_lat_min)
            .def_prop_ro("p_lat_max", &ReachNode::p_lat_max)
            .def_prop_ro("is_empty", &ReachNode::is_empty)
            .def_prop_ro("position_rectangle", &ReachNode::position_rectangle)
            .def_prop_ro("list_nodes_parent", &ReachNode::vec_nodes_parent)
            .def_prop_ro("list_nodes_child", &ReachNode::vec_nodes_child)
            .def_ro("id", &ReachNode::id)
            .def_ro("step", &ReachNode::step)
            .def_ro("polygon_lon", &ReachNode::polygon_lon)
            .def_ro("polygon_lat", &ReachNode::polygon_lat)
            .def("intersect_in_position_domain", &ReachNode::intersect_in_position_domain,
                 "p_lon_min"_a = -std::numeric_limits<double>::infinity(),
                 "p_lat_min"_a = -std::numeric_limits<double>::infinity(),
                 "p_lon_max"_a = std::numeric_limits<double>::infinity(),
                 "p_lat_max"_a = std::numeric_limits<double>::infinity())
            .def("intersect_in_velocity_domain", &ReachNode::intersect_in_velocity_domain,
                 "v_lon_min"_a = -std::numeric_limits<double>::infinity(),
                 "v_lat_min"_a = -std::numeric_limits<double>::infinity(),
                 "v_lon_max"_a = std::numeric_limits<double>::infinity(),
                 "v_lat_max"_a = std::numeric_limits<double>::infinity())
            .def("clone", &ReachNode::clone)
            .def("__repr__", [](ReachNode const &node) {
                return "(" + std::to_string(node.p_lon_min()) + ", " + std::to_string(node.p_lat_min())
                       + ", " + std::to_string(node.p_lon_max()) + ", " + std::to_string(node.p_lat_max()) + ")";
            });
}

void export_configuration(nb::module_ &m) {
    nb::enum_<CoordinateSystem>(m, "CoordinateSystem")
            .value("CARTESIAN", CoordinateSystem::CARTESIAN)
            .value("CURVILINEAR", CoordinateSystem::CURVILINEAR)
            .export_values();

    nb::enum_<ReferencePoint>(m, "ReferencePoint")
            .value("CENTER", ReferencePoint::CENTER)
            .value("REAR", ReferencePoint::REAR)
            .export_values();

    nb::class_<Configuration>(m, "Configuration")
            .def(nb::init<>())
            .def_rw("general", &Configuration::config_general)
            .def_rw("vehicle", &Configuration::config_vehicle)
            .def_rw("planning", &Configuration::config_planning)
            .def_rw("reachable_set", &Configuration::config_reachable_set)
            .def_rw("debug", &Configuration::config_debug);

    nb::class_<GeneralConfiguration>(m, "GeneralConfiguration")
            .def(nb::init<>())
            .def_rw("name_scenario", &GeneralConfiguration::name_scenario)
            .def_rw("path_scenarios", &GeneralConfiguration::path_scenarios);

    nb::class_<VehicleConfiguration>(m, "VehicleConfiguration")
            .def(nb::init<>())
            .def_rw("ego", &VehicleConfiguration::ego)
            .def_rw("other", &VehicleConfiguration::other);

    nb::class_<Ego>(m, "Ego")
            .def(nb::init<>())
            .def_rw("id_type_vehicle", &Ego::id_type_vehicle)
            .def_rw("id_vehicle", &Ego::id_vehicle)
            .def_rw("length", &Ego::length)
            .def_rw("width", &Ego::width)
            .def_rw("radius_disc", &Ego::radius_disc)
            .def_rw("circle_distance", &Ego::circle_distance)
            .def_rw("wheelbase", &Ego::wheelbase)
            .def_rw("v_lon_min", &Ego::v_lon_min)
            .def_rw("v_lon_max", &Ego::v_lon_max)
            .def_rw("v_lat_min", &Ego::v_lat_min)
            .def_rw("v_lat_max", &Ego::v_lat_max)
            .def_rw("a_lon_min", &Ego::a_lon_min)
            .def_rw("a_lon_max", &Ego::a_lon_max)
            .def_rw("a_lat_min", &Ego::a_lat_min)
            .def_rw("a_lat_max", &Ego::a_lat_max)
            .def_rw("a_max", &Ego::a_max)
            .def_rw("t_react", &Ego::t_react)
            .def_rw("fov", &Ego::fov);

    nb::class_<Other>(m, "Other")
            .def(nb::init<>())
            .def_rw("id_type_vehicle", &Other::id_type_vehicle)
            .def_rw("id_vehicle", &Other::id_vehicle)
            .def_rw("length", &Other::length)
            .def_rw("width", &Other::width)
            .def_rw("radius_disc", &Other::radius_disc)
            .def_rw("circle_distance", &Other::circle_distance)
            .def_rw("wheelbase", &Other::wheelbase)
            .def_rw("v_lon_min", &Other::v_lon_min)
            .def_rw("v_lon_max", &Other::v_lon_max)
            .def_rw("v_lat_min", &Other::v_lat_min)
            .def_rw("v_lat_max", &Other::v_lat_max)
            .def_rw("a_lon_min", &Other::a_lon_min)
            .def_rw("a_lon_max", &Other::a_lon_max)
            .def_rw("a_lat_min", &Other::a_lat_min)
            .def_rw("a_lat_max", &Other::a_lat_max)
            .def_rw("a_max", &Other::a_max)
            .def_rw("t_react", &Other::t_react);

    nb::class_<PlanningConfiguration>(m, "PlanningConfiguration")
            .def(nb::init<>())
            .def_rw("dt", &PlanningConfiguration::dt)
            .def_rw("step_start", &PlanningConfiguration::step_start)
            .def_rw("steps_computation", &PlanningConfiguration::steps_computation)
            .def_rw("p_lon_initial", &PlanningConfiguration::p_lon_initial)
            .def_rw("p_lat_initial", &PlanningConfiguration::p_lat_initial)
            .def_rw("uncertainty_p_lon", &PlanningConfiguration::uncertainty_p_lon)
            .def_rw("uncertainty_p_lat", &PlanningConfiguration::uncertainty_p_lat)
            .def_rw("v_lon_initial", &PlanningConfiguration::v_lon_initial)
            .def_rw("v_lat_initial", &PlanningConfiguration::v_lat_initial)
            .def_rw("uncertainty_v_lon", &PlanningConfiguration::uncertainty_v_lon)
            .def_rw("uncertainty_v_lat", &PlanningConfiguration::uncertainty_v_lat)
            .def_rw("step_start", &PlanningConfiguration::step_start)
            .def_rw("id_lanelet_initial", &PlanningConfiguration::id_lanelet_initial)
            .def_rw("coordinate_system", &PlanningConfiguration::coordinate_system)
            .def_rw("reference_point", &PlanningConfiguration::reference_point)
            .def_rw("CLCS", &PlanningConfiguration::CLCS);

    nb::class_<ReachableSetConfiguration>(m, "ReachableSetConfiguration")
            .def(nb::init<>())
            .def_rw("mode_repartition", &ReachableSetConfiguration::mode_repartition)
            .def_rw("mode_inflation", &ReachableSetConfiguration::mode_inflation)
            .def_rw("size_grid", &ReachableSetConfiguration::size_grid)
            .def_rw("size_grid_2nd", &ReachableSetConfiguration::size_grid_2nd)
            .def_rw("radius_terminal_split", &ReachableSetConfiguration::radius_terminal_split)
            .def_rw("num_threads", &ReachableSetConfiguration::num_threads)
            .def_rw("prune_nodes", &ReachableSetConfiguration::prune_nodes)
            .def_rw("lut_lon_enlargement", &ReachableSetConfiguration::lut_lon_enlargement)
            .def_rw("rasterize_obstacles", &ReachableSetConfiguration::rasterize_obstacles);

    nb::class_<DebugConfiguration>(m, "DebugConfiguration")
            .def(nb::init<>())
            .def_rw("verbose_mode", &DebugConfiguration::verbose_mode)
            .def_rw("measure_time", &DebugConfiguration::measure_time);
}