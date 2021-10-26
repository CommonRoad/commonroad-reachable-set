#include <iostream>
#include <chrono>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <yaml-cpp/yaml.h>

#include "geometry/curvilinear_coordinate_system.h"
#include "collision/collision_checker.h"
#include "reachset/common/reachable_set_interface.hpp"
#include "reachset/common/collision_checker.hpp"
#include "reachset/semantic/data_structure/traffic_rule.hpp"
#include "reachset/semantic/data_structure/semantic_model.hpp"

using namespace reach;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
namespace py = pybind11;

using CollisionCheckerPtr = collision::CollisionCheckerPtr;

int main() {
    // start the python interpreter and keep it alive
    py::scoped_interpreter python{};

    // ======== settings
    string path_root = "/home/edmond/Softwares/commonroad/commonroad-reachable-set/";
    YAML::Node node_config = YAML::LoadFile("../../configurations/config_cpp.yaml");
    auto name_scenario = node_config["name_scenario"].as<string>();
    auto visualize_results = node_config["visualize_results"].as<bool>();
    auto save_fig = node_config["save_fig"].as<bool>();
    auto save_gif = node_config["save_gif"].as<bool>();
    auto plot_refined = node_config["plot_refined"].as<bool>();

    // append path to interpreter
    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("append")(path_root);
    sys.attr("path").attr("append")("/home/edmond/Softwares/commonroad/spot-cpp");

    // ======== configuration object via python ConfigurationBuilder
    auto cls_ConfigurationBuilder_py =
            py::module_::import("commonroad_reachset.common.configuration_builder").attr("ConfigurationBuilder");
    cls_ConfigurationBuilder_py.attr("set_root_path")(path_root);
    auto obj_config_py = cls_ConfigurationBuilder_py.attr("build_configuration")(name_scenario);
    auto config = obj_config_py.attr("convert_to_cpp_configuration")().cast<ConfigurationPtr>();

    // ======== traffic rule object via python TrafficRule
    auto cls_TrafficRule_py = py::module_::import("commonroad_reachset.semantic.data_structure.traffic_rule").attr(
            "TrafficRule");
    auto obj_traffic_rule_py = cls_TrafficRule_py(obj_config_py);
    auto traffic_rule = TrafficRule(obj_traffic_rule_py);

    // ======== semantic model object via python SemanticModel
    auto cls_SemanticModel_py =
            py::module_::import("commonroad_reachset.semantic.data_structure.semantic_model").attr("SemanticModel");
    auto obj_semantic_model_py = cls_SemanticModel_py(obj_config_py, obj_traffic_rule_py);
    auto semantic_model = make_shared<SemanticModel>(obj_semantic_model_py);

    // ======== CurvilinearCoordinateSystem
    auto CLCS = make_shared<geometry::CurvilinearCoordinateSystem>(
            obj_config_py.attr("planning").attr("reference_path").cast<geometry::EigenPolyline>(),
            25.0, 0.1);

    // ======== collision checker via python collision checker
    auto cls_CollisionChecker_py =
            py::module_::import("commonroad_reachset.common.collision_checker").attr("CollisionChecker");
    auto obj_collision_checker_py = cls_CollisionChecker_py(obj_config_py);
    auto collision_checker = create_curvilinear_collision_checker(
            obj_collision_checker_py.attr("list_vertices_polygons_static").cast<vector<Polyline>>(),
            obj_collision_checker_py.attr(
                    "dict_time_step_to_list_vertices_polygons_dynamic").cast<map<int, vector<Polyline>>>(),
            CLCS,
            obj_config_py.attr("vehicle").attr("ego").attr("radius_disc").cast<double>(),
            4);

    // ======== ReachableSetInterface
    auto reach_interface = ReachableSetInterface::semantic(config, collision_checker, semantic_model);

    auto start = high_resolution_clock::now();
    reach_interface.compute();
    auto end = high_resolution_clock::now();
    cout << "Computation time: " << duration_cast<milliseconds>(end - start).count() << "ms" << endl;

    // ======== model checking with PyNuSMV
    auto cls_ModelChecker_py =
            py::module_::import("commonroad_reachset.semantic.model_checker").attr("ModelChecker");
    auto obj_model_checker_py = cls_ModelChecker_py(obj_config_py, reach_interface, obj_traffic_rule_py);
    reach_interface = obj_model_checker_py.attr("check_model")().cast<ReachableSetInterface>();

    // ======== maneuver extraction

    // ======== visualization of results
    if (visualize_results) {
        auto utils_visualization = py::module_::import("commonroad_reachset.common.utility.visualization");
        utils_visualization.attr("draw_scenario_with_reach_cpp")(obj_config_py, reach_interface,
                                                                 py::arg("save_gif") = save_gif,
                                                                 py::arg("save_fig") = save_fig,
                                                                 py::arg("plot_refined") = plot_refined);
    }
    cout << "Done." << endl;

    return 0;
}