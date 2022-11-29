#include <iostream>
#include <chrono>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "collision/collision_checker.h"
#include "geometry/curvilinear_coordinate_system.h"
#include "reachset/data_structure/reach/reach_set.hpp"
#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reachset/utility/collision_checker.hpp"

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
    //string name_scenario = "USA_US101-6_1_T-1";
    //string name_scenario = "DEU_Test-1_1_T-1";
    string name_scenario = "ARG_Carcarana-1_1_T-1";
    //string name_scenario = "ZAM_Tjunction-1_313_T-1";

    // append root path to python interpreter
    string path_root = "/home/edmond/Softwares/commonroad/commonroad-reach/";
    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("append")(path_root);

    // ======== create configuration
    auto cls_ConfigurationBuilder_py =
            py::module_::import("commonroad_reach.data_structure.configuration_builder").attr(
                    "ConfigurationBuilder");
    auto obj_config_py = cls_ConfigurationBuilder_py.attr("build_configuration")(name_scenario, path_root);
    obj_config_py.attr("update")();
    obj_config_py.attr("print_configuration_summary")();
    auto config = obj_config_py.attr("convert_to_cpp_configuration")().cast<ConfigurationPtr>();

    // ======== collision checker
    auto cls_CollisionChecker_py =
            py::module_::import("commonroad_reach.data_structure.collision_checker").attr("CollisionChecker");
    auto obj_collision_checker_py = cls_CollisionChecker_py(obj_config_py);
    auto collision_checker =
            obj_collision_checker_py.attr("cpp_collision_checker").cast<CollisionCheckerPtr>();

    // ======== compute reachable sets
    auto reachable_set = ReachableSet(config, collision_checker);
    auto start = high_resolution_clock::now();
    cout << "Computing reachable sets..." << endl;
    reachable_set.compute();
    auto end = high_resolution_clock::now();
    cout << "Computation time: " << duration_cast<milliseconds>(end - start).count() << "ms" << endl;

    // ======== plot computation results
    auto util_visualization = py::module_::import("commonroad_reach.utility.visualization");
    // requires opencv-python==4.3.0.36
    //util_visualization.attr("plot_scenario_with_reachable_sets_cpp")(reachable_set, obj_config_py);

    return 0;
}