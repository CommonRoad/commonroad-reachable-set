#include "reachset/data_structure/configuration.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace reach;

GeneralConfiguration::GeneralConfiguration(YAML::Node const& node) {
    auto node_general = node["general"];

    name_scenario = node_general["name_scenario"].as<string>();
    path_scenario = path_scenarios + name_scenario + ".xml";
}

Ego::Ego(YAML::Node const& node) {
    auto node_vehicle = node["vehicle"]["ego"];

    id_type_vehicle = node_vehicle["id_type_vehicle"].as<int>();
    id_vehicle = 0;

    length = node_vehicle["length"].as<double>();
    width = node_vehicle["width"].as<double>();
    radius_disc = 1.2;
    circle_distance = 2.0;
    wheelbase = 2.0;

    v_lon_min = node_vehicle["v_lon_min"].as<double>();
    v_lon_max = node_vehicle["v_lon_max"].as<double>();
    v_lat_min = node_vehicle["v_lat_min"].as<double>();
    v_lat_max = node_vehicle["v_lat_max"].as<double>();

    a_lon_min = node_vehicle["a_lon_min"].as<double>();
    a_lon_max = node_vehicle["a_lon_max"].as<double>();
    a_lat_min = node_vehicle["a_lat_min"].as<double>();
    a_lat_max = node_vehicle["a_lat_max"].as<double>();
    a_max = node_vehicle["a_max"].as<double>();
}

Other::Other(YAML::Node const& node) {
    auto node_vehicle = node["vehicle"]["other"];

    id_type_vehicle = node_vehicle["id_type_vehicle"].as<int>();
    id_vehicle = 0;

    length = node_vehicle["length"].as<double>();
    width = node_vehicle["width"].as<double>();
    radius_disc = 1.2;
    circle_distance = 2.0;
    wheelbase = 2.0;

    v_lon_min = node_vehicle["v_lon_min"].as<double>();
    v_lon_max = node_vehicle["v_lon_max"].as<double>();
    v_lat_min = node_vehicle["v_lat_min"].as<double>();
    v_lat_max = node_vehicle["v_lat_max"].as<double>();

    a_lon_min = node_vehicle["a_lon_min"].as<double>();
    a_lon_max = node_vehicle["a_lon_max"].as<double>();
    a_lat_min = node_vehicle["a_lat_min"].as<double>();
    a_lat_max = node_vehicle["a_lat_max"].as<double>();
    a_max = node_vehicle["a_max"].as<double>();
}

VehicleConfiguration::VehicleConfiguration(YAML::Node const& node) {
    ego = Ego(node);
    other = Other(node);
}

PlanningConfiguration::PlanningConfiguration(YAML::Node const& node) {
    auto node_planning = node["planning"];

    dt = node_planning["dt"].as<double>();
    step_start = node_planning["step_start"].as<int>();
    steps_computation = node_planning["steps_computation"].as<int>();

    p_lon_initial = 0;
    p_lat_initial = 0;
    uncertainty_p_lon = node_planning["uncertainty_p_lon"].as<double>();
    uncertainty_p_lat = node_planning["uncertainty_p_lat"].as<double>();

    v_lon_initial = 0;
    v_lat_initial = 0;
    uncertainty_v_lon = node_planning["uncertainty_v_lon"].as<double>();
    uncertainty_v_lat = node_planning["uncertainty_v_lat"].as<double>();

    id_lanelet_initial = 0;

    auto _coordinate_system = node_planning["coordinate_system"].as<string>();
    if (_coordinate_system == "CVLN") coordinate_system = CoordinateSystem::CARTESIAN;
    else if (_coordinate_system == "CART") coordinate_system = CoordinateSystem::CURVILINEAR;
    else throw std::invalid_argument("<PlanningConfiguration> Invalid coordinate system.");

    auto _reference_point = node_planning["reference_point"].as<string>();
    if (_reference_point == "CENTER") reference_point = ReferencePoint::CENTER;
    else if (_reference_point == "REAR") reference_point = ReferencePoint::REAR;
    else throw std::invalid_argument("<PlanningConfiguration> Invalid reference point.");
}


ReachableSetConfiguration::ReachableSetConfiguration(YAML::Node const& node) {
    auto node_reachable_set = node["reachable_set"];

    mode_repartition = node_reachable_set["mode_repartition"].as<int>();
    mode_inflation = node_reachable_set["mode_inflation"].as<int>();
    size_grid = node_reachable_set["size_grid"].as<double>();
    size_grid_2nd = node_reachable_set["size_grid_2nd"].as<double>();
    radius_terminal_split = node_reachable_set["radius_terminal_split"].as<double>();
    num_threads = node_reachable_set["num_threads"].as<int>();
}

DebugConfiguration::DebugConfiguration(YAML::Node const& node) {
    auto node_debug = node["debug"];
}

Configuration::Configuration(YAML::Node const& node) {
    config_general = GeneralConfiguration(node);
    config_vehicle = VehicleConfiguration(node);
    config_planning = PlanningConfiguration(node);
    config_reachable_set = ReachableSetConfiguration(node);
    config_debug = DebugConfiguration(node);
}

ConfigurationPtr Configuration::load_configuration(string const& file_yaml) {
    YAML::Node node = YAML::LoadFile(file_yaml);
    auto config = Configuration(node);
    return make_shared<Configuration>(node);
}
