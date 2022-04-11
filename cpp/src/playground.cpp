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
#include "reachset/utility/miscellaneous.hpp"

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

    // append path to interpreter
    string path_root = "/home/edmond/Softwares/commonroad/commonroad-reachable-set/";
    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("append")(path_root);

    // ======== configuration from python ConfigurationBuilder
    auto cls_ConfigurationBuilder_py =
            py::module_::import("commonroad_reach.data_structure.configuration_builder").attr(
                    "ConfigurationBuilder");
    auto obj_config_py = cls_ConfigurationBuilder_py.attr("build_configuration")(name_scenario);
    obj_config_py.attr("print_configuration_summary");
    auto config = obj_config_py.attr("convert_to_cpp_configuration")().cast<ConfigurationPtr>();

    // ======== curvilinear coordinate system
    auto CLCS = make_shared<geometry::CurvilinearCoordinateSystem>(
            obj_config_py.attr("planning").attr("reference_path").cast<geometry::EigenPolyline>(), 25.0, 0.1);

    // ======== collision checker from python collision checker
    auto cls_CollisionChecker_py =
            py::module_::import("commonroad_reach.data_structure.collision_checker").attr("CollisionChecker");
    auto obj_collision_checker_py = cls_CollisionChecker_py(obj_config_py);
    auto collision_checker =
            obj_collision_checker_py.attr("cpp_collision_checker").cast<CollisionCheckerPtr>();

    // ======== reachable set computation
    auto reachable_set = ReachableSet(config, collision_checker);
    auto start = high_resolution_clock::now();
    cout << "Computing reachable sets..." << endl;
    reachable_set.compute();
    auto end = high_resolution_clock::now();
    cout << "Computation time: " << duration_cast<milliseconds>(end - start).count() << "ms" << endl;

    // //======== visualization of results
    //auto utils_visualization = py::module_::import("commonroad_reach.utility.visualization");
    //utils_visualization.attr("draw_scenario_with_reach_cpp")(obj_config_py, reach_interface,
    //                                                         py::arg("save_gif") = true,
    //                                                         py::arg("save_fig") = false);
    return 0;
}