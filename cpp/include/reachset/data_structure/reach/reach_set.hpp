#pragma once

#include "reachset/utility/shared_include.hpp"
#include "reachset/data_structure/reach/reach_polygon2.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"
#include "reachset/data_structure/configuration.hpp"
#include "collision/collision_checker.h"
#include <omp.h>

using CollisionCheckerPtr = collision::CollisionCheckerPtr;

namespace reach {
/// Reachable set representation for the ego vehicle.
class ReachableSet {
private:
    bool _reachable_set_computed{false};
    bool _prune_reachable_set{false};
    bool _pruned{false};
    std::vector<int> _vec_time_steps_computed{0};

    void _initialize();

    /// Initializes the zero-state polygons of the system.
    void _initialize_zero_state_polygons();

    /// Drivable area at the initial time step.
    /// Constructed directly from the config file.
    std::vector<ReachPolygon2Ptr> _construct_initial_drivable_area() const;

    /// Reachable set at the initial time step.
    /// Vertices of the longitudinal and lateral polygons are constructed directly from the config file.
    std::vector<ReachNodePtr> _construct_initial_reachable_set() const;

    void _compute_drivable_area_at_time_step(int const& time_step);

    void _compute_reachable_set_at_time_step(int const& time_step);

    std::vector<ReachNodePtr> _propagate_reachable_set(const std::vector<ReachNodePtr>& vec_nodes);

    void _prune_nodes_not_reaching_final_time_step() { _pruned = true; }

public:
    explicit ReachableSet(ConfigurationPtr config);

    ReachableSet(ConfigurationPtr config, CollisionCheckerPtr collision_checker);

    ConfigurationPtr config;
    CollisionCheckerPtr collision_checker;

    int time_step_start{};
    int time_step_end{};
    std::map<int, std::vector<ReachPolygon2Ptr>> map_time_to_drivable_area;
    std::map<int, std::vector<ReachNodePtr>> map_time_to_base_set_propagated;
    std::map<int, std::vector<ReachNodePtr>> map_time_to_reachable_set;
    std::map<int, std::vector<ReachNodePtr>> map_time_to_reachable_set_pruned;

    ReachPolygon2Ptr polygon_zero_state_lon;
    ReachPolygon2Ptr polygon_zero_state_lat;

    inline std::vector<ReachPolygon2Ptr> drivable_area_at_time_step(int const& time_step) {
        if (find(_vec_time_steps_computed.begin(), _vec_time_steps_computed.end(), time_step)
            != _vec_time_steps_computed.end()) {
            cout << "Given time step for drivable area retrieval is out of range." << endl;
            return {};
        } else return map_time_to_drivable_area[time_step];
    }

    inline std::vector<ReachNodePtr> reachable_set_at_time_step(int const& time_step) {
        if (find(_vec_time_steps_computed.begin(), _vec_time_steps_computed.end(), time_step)
            != _vec_time_steps_computed.end()) {
            cout << "Given time step for reachable set retrieval is out of range." << endl;
            return {};
        } else return map_time_to_reachable_set[time_step];
    }

    inline std::map<int, std::vector<ReachPolygon2Ptr>> drivable_area() const {
        return map_time_to_drivable_area;
    }

    inline std::map<int, std::vector<ReachNodePtr>> reachable_set() const {
        return map_time_to_reachable_set;
    }

    void compute(int step_start = 0, int step_end = 0);
};

using ReachableSetPtr = shared_ptr<ReachableSet>;
}