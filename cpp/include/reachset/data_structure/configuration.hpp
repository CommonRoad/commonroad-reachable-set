#pragma once

#include "reachset/utility/shared_include.hpp"
#include "reachset/utility/shared_using.hpp"
#include "reachset/utility/lut_longitudinal_enlargement.hpp"
#include <yaml-cpp/yaml.h>
#include "geometry/curvilinear_coordinate_system.h"

namespace reach {

enum class CoordinateSystem {
    /// Cartesian coordinate system
    CARTESIAN,
    /// Curvilinear coordinate system
    CURVILINEAR
};

enum class ReferencePoint {
    /// Center point of the vehicle
    CENTER,
    /// Rear axle of the vehicle
    REAR
};

/// Struct storing general configurations.
struct GeneralConfiguration {
    string name_scenario{};
    string path_scenarios{};
    string path_scenario{};

    GeneralConfiguration() = default;

    explicit GeneralConfiguration(YAML::Node const& node);
};

/// Struct storing configurations for ego vehicle.
struct Ego {
    int id_type_vehicle{};
    int id_vehicle{};
    // physical profile
    double length{};
    double width{};
    double radius_disc{};
    double circle_distance{};
    double wheelbase{};
    // velocity profile
    double v_lon_min{};
    double v_lon_max{};
    double v_lat_min{};
    double v_lat_max{};
    // acceleration profile
    double a_lon_min{};
    double a_lon_max{};
    double a_lat_min{};
    double a_lat_max{};
    double a_max{};
    // reaction time
    double t_react{};
    double fov{};

    Ego() = default;

    explicit Ego(YAML::Node const& node);
};

/// Struct storing configurations for other vehicles.
struct Other {
    int id_type_vehicle{};
    int id_vehicle{};
    // physical profile
    double length{};
    double width{};
    double radius_disc{};
    double circle_distance{};
    double wheelbase{};
    // velocity profile
    double v_lon_min{};
    double v_lon_max{};
    double v_lat_min{};
    double v_lat_max{};
    // acceleration profile
    double a_lon_min{};
    double a_lon_max{};
    double a_lat_min{};
    double a_lat_max{};
    double a_max{};
    // reaction time
    double t_react{};

    Other() = default;

    explicit Other(YAML::Node const& node);
};

/// Struct storing vehicle configurations.
struct VehicleConfiguration {
    Ego ego;
    Other other;

    VehicleConfiguration() = default;

    explicit VehicleConfiguration(YAML::Node const& node);
};

/// Struct storing planning configurations.
struct PlanningConfiguration {
    double dt{};
    int step_start{};
    int steps_computation{};
    // initial positions
    double p_lon_initial{};
    double p_lat_initial{};
    double uncertainty_p_lon{};
    double uncertainty_p_lat{};
    // initial velocities
    double v_lon_initial{};
    double v_lat_initial{};
    double uncertainty_v_lon{};
    double uncertainty_v_lat{};
    // initial lanelet id
    int id_lanelet_initial{};

    CoordinateSystem coordinate_system{CoordinateSystem::CURVILINEAR};
    ReferencePoint reference_point{ReferencePoint::CENTER};

    // curvilinear coordinate system object
    std::shared_ptr<const geometry::CurvilinearCoordinateSystem> CLCS;

    PlanningConfiguration() = default;

    explicit PlanningConfiguration(YAML::Node const& node);
};

/// Struct storing reachable set configurations.
struct ReachableSetConfiguration {
    // mode for drivable area repartition
    int mode_repartition{};
    // mode for considering the shape of the ego vehicle
    int mode_inflation{};
    // grid size for repartitioning rectangles
    double size_grid{};
    // grid size for repartitioning rectangles (second time)
    double size_grid_2nd{};
    // terminal radius for splitting of rectangles
    double radius_terminal_split{};
    // number of threads in parallel computation
    int num_threads{};
    // flag whether to prune reach nodes not reaching the final step
    bool prune_nodes{};
    // flag whether to rasterize obstacles when using CVLN frame (reduces over-approximation after conversion to CVLN)
    bool rasterize_obstacles{};
    // look-up table for longitudinal enlargement for collision checking
    LUTLongitudinalEnlargementConstPtr lut_lon_enlargement;

    ReachableSetConfiguration() = default;

    explicit ReachableSetConfiguration(YAML::Node const& node);
};

/// Struct storing debugging configurations.
struct DebugConfiguration {
    bool verbose_mode{false};
    bool measure_time{false};

    DebugConfiguration() = default;

    explicit DebugConfiguration(YAML::Node const& node);
};

/// Struct storing all configurations.
struct Configuration {
    GeneralConfiguration config_general{};
    VehicleConfiguration config_vehicle{};
    PlanningConfiguration config_planning{};
    ReachableSetConfiguration config_reachable_set{};
    DebugConfiguration config_debug{};

    Configuration() = default;

    explicit Configuration(YAML::Node const& node);

    inline GeneralConfiguration& general() { return config_general; };

    inline VehicleConfiguration& vehicle() { return config_vehicle; };

    inline PlanningConfiguration& planning() { return config_planning; };

    inline ReachableSetConfiguration& reachable_set() { return config_reachable_set; };

    inline DebugConfiguration& debug() { return config_debug; };

    /// Loads configuration from the given yaml file.
    static std::shared_ptr<Configuration> load_configuration(std::string const& file_yaml);
};

using ConfigurationPtr = std::shared_ptr<Configuration>;
}

