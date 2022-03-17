#include <iostream>
#include <chrono>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "collision/collision_checker.h"
#include "geometry/curvilinear_coordinate_system.h"
#include "reachset/data_structure/reach/reach_interface.hpp"
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
    string name_scenario = "DEU_Test-1_1_T-1";
    //string name_scenario = "USA_US101-15_1_T-1";
    //string name_scenario = "ARG_Carcarana-1_1_T-1";
    //string name_scenario = "ZAM_Tjunction-1_313_T-1";
    string path_root = "/home/edmond/Softwares/commonroad/commonroad-reachable-set/";

    // append path to interpreter
    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("append")(path_root);

    // ======== configuration via python ConfigurationBuilder
    auto cls_ConfigurationBuilder_py =
            py::module_::import("commonroad_reach.data_structure.configuration_builder").attr(
                    "ConfigurationBuilder");
    auto obj_config_py = cls_ConfigurationBuilder_py.attr("build_configuration")(name_scenario);
    auto config = obj_config_py.attr("convert_to_cpp_configuration")().cast<ConfigurationPtr>();

    // ======== CurvilinearCoordinateSystem
    auto CLCS = make_shared<geometry::CurvilinearCoordinateSystem>(
            obj_config_py.attr("planning").attr("reference_path").cast<geometry::EigenPolyline>(), 25.0, 0.1);

    // ======== collision checker via python collision checker
    auto cls_CollisionChecker_py =
            py::module_::import("commonroad_reach.data_structure.collision_checker_cpp").attr("CppCollisionChecker");
    auto obj_collision_checker_py = cls_CollisionChecker_py(obj_config_py);

    auto vec_vertices_polygons_static =
            obj_collision_checker_py.attr("list_vertices_polygons_static").cast<vector<Polyline>>();
    auto map_time_to_vec_vertices_polygons_dynamic = obj_collision_checker_py.attr(
            "dict_time_to_list_vertices_polygons_dynamic").cast<map<int, vector<Polyline>>>();
    auto radius_disc = obj_config_py.attr("vehicle").attr("ego").attr("radius_disc").cast<double>();

    auto collision_checker =
            create_curvilinear_collision_checker(vec_vertices_polygons_static,
                                                 map_time_to_vec_vertices_polygons_dynamic,
                                                 CLCS, radius_disc, 4);

    // ======== ReachableSetInterface
    auto reach_interface = ReachableSetInterface(config, collision_checker);

    //print_collision_checker(reach_interface.reachable_set->collision_checker());

    auto start = high_resolution_clock::now();
    cout << "Computing reachable sets..." << endl;
    reach_interface.compute_reachable_sets();
    auto end = high_resolution_clock::now();
    cout << "Computation time: " << duration_cast<milliseconds>(end - start).count() << "ms" << endl;
    collision_checker->timeSlice(10);

    // //======== visualization of results
    //auto utils_visualization = py::module_::import("commonroad_reach.utility.visualization");
    //utils_visualization.attr("draw_scenario_with_reach_cpp")(obj_config_py, reach_interface,
    //                                                         py::arg("save_gif") = true,
    //                                                         py::arg("save_fig") = false);

    cout << "Done." << endl;

    return 0;
}