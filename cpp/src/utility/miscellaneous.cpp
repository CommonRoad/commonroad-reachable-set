#include "reachset/utility/miscellaneous.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace collision;
using namespace reach;

ConfigurationPtr create_mock_configuration() {
    auto config = make_shared<Configuration>();

    config->general().name_scenario = "DEU_Test-1_1_T-1";
    config->general().path_scenarios = "/home/julian/TUM/MasterThesis/CodeBase/cooperative-motion-planning/reach/scenarios/";
    config->general().path_scenario = "/home/julian/TUM/MasterThesis/CodeBase/cooperative-motion-planning/reach/configuration/";

    config->vehicle().ego.id_type_vehicle = 8;
    config->vehicle().ego.id_vehicle = 0;

    config->vehicle().ego.length = 4.5;
    config->vehicle().ego.width = 2.0;
    config->vehicle().ego.wheelbase = 3.0;
    config->vehicle().ego.radius_disc = 1.3;

    config->vehicle().ego.a_lon_max = 2.0;
    config->vehicle().ego.a_lon_min = -6.0;
    config->vehicle().ego.a_lat_max = 2.0;
    config->vehicle().ego.a_lat_min = -2.0;
    config->vehicle().ego.a_max = 8.0;
    config->vehicle().ego.v_lon_max = 20.0;
    config->vehicle().ego.v_lon_min = -5.0;
    config->vehicle().ego.v_lat_max = 4.0;
    config->vehicle().ego.v_lat_min = -4.0;
    config->vehicle().ego.t_react = 0.5;

    config->planning().dt = 0.2;
    config->planning().id_lanelet_initial = 0;
    config->planning().time_step_start = 0;
    config->planning().time_steps_computation = 20;
    config->planning().uncertainty_p_lat = 0.01;
    config->planning().uncertainty_p_lon = 0.01;
    config->planning().uncertainty_v_lat = 0.01;
    config->planning().uncertainty_v_lon = 0.01;
    config->planning().v_lat_initial = 0.0;
    config->planning().v_lon_initial = 12.0;
    config->planning().reference_point = ReferencePoint::CENTER;

    config->planning().coordinate_system = CoordinateSystem::CURVILINEAR;
    if (config->planning().coordinate_system == CoordinateSystem::CURVILINEAR) {
        config->planning().p_lat_initial = 0.1;
        config->planning().p_lon_initial = 35.1;
    } else {
        config->planning().p_lat_initial = 2.1;
        config->planning().p_lon_initial = 35.1;
    }

    config->reachable_set().size_grid = 0.2;
    config->reachable_set().size_grid_2nd = 0.1;
    config->reachable_set().radius_terminal_split = 0.5;

    config->debug().verbose_mode = false;
    config->debug().measure_time = false;

    return config;
}

TimeVariantCollisionObjectPtr create_tvo_from_specs(double const& length, double const& width,
                                                    tuple<double, double> const& p_init, double const& v_init,
                                                    double const& o_init, int const& time_steps) {
    auto tvo = std::make_shared<TimeVariantCollisionObject>(0);
    auto x{0.0}, y{0.0}, dt{0.1};

    for (int time_step = 0; time_step < time_steps; ++time_step) {
        x = std::get<0>(p_init) + dt * time_step * v_init;
        y = std::get<1>(p_init);

        tvo->appendObstacle(CollisionObjectConstPtr(
                new RectangleOBB(length / 2.0, width / 2.0, o_init, Eigen::Vector2d(x, y))));
    }

    return tvo;
}


void print_collision_checker(CollisionCheckerPtr const& cc) {
    auto vec_obstacles = cc->getObstacles();

    for (auto const& obs: vec_obstacles) {
        if (obs->getCollisionObjectClass() == collision::OBJ_CLASS_TVOBSTACLE) {
            cout << "TVO:" << endl;
            for (int i = 0; i < 40; ++i) {
                auto obj_at_time = obs->timeSlice(i, obs);
                auto aabb = obj_at_time->getAABB();
                cout << aabb->r_x() << ", " << aabb->r_y() << endl;

                cout << "\t" << i << ": " << aabb->center_x() << ", " << aabb->center_y() << endl;
            }
        }
    }
}