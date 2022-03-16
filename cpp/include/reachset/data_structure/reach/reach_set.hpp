#pragma once

#include "reachset/utility/shared_include.hpp"
#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"
#include "reachset/data_structure/configuration.hpp"
#include "collision/collision_checker.h"
#include <omp.h>

using CollisionCheckerPtr = collision::CollisionCheckerPtr;

namespace reach {
/// Reachable set representation for the ego vehicle."""
class ReachableSet {
private:
    bool _prune_reachable_set{false};
    bool _pruned{false};
    std::vector<int> _vec_time_steps_computed{0};


public:
    explicit ReachableSet(ConfigurationPtr config);

    ReachableSet(ConfigurationPtr config, CollisionCheckerPtr collision_checker);

    ConfigurationPtr config;
    CollisionCheckerPtr collision_checker;

    int time_step_start{};
    int time_step_end{};
    std::map<int, std::vector<ReachPolygonPtr>> map_time_to_base_set_propagated;
    std::map<int, std::vector<ReachPolygonPtr>> map_time_to_drivable_area;
    std::map<int, std::vector<ReachNodePtr>> map_time_to_reachable_set;
    std::map<int, std::vector<ReachNodePtr>> map_time_to_reachable_set_pruned;

    ReachPolygonPtr polygon_zero_state_lon;
    ReachPolygonPtr polygon_zero_state_lat;

    std::vector<ReachPolygonPtr> drivable_area_at_time_step(int const& time_step);

    std::vector<ReachNodePtr> reachable_set_at_time_step(int const& time_step);
    /// Drivable area at the initial time step.
    /// Constructed directly from the config file.
    std::vector<ReachPolygonPtr> construct_initial_drivable_area() const;

    /// Reachable set at the initial time step.
    /// Vertices of the longitudinal and lateral polygons are constructed directly from the config file.
    std::vector<ReachNodePtr> construct_initial_reachable_set() const;

    /// Initializes the zero-state polygons of the system.
    void initialize_zero_state_polygons();

    void compute_reachable_sets(int const& step_start, int const& step_end);

    std::tuple<std::vector<ReachPolygonPtr>, std::vector<ReachNodePtr>>
    compute_drivable_area_at_time_step(int const& time_step,
                                       std::vector<ReachNodePtr> const& reachable_set_time_step_previous);

    /// Propagates the nodes of the reachable set from the previous time step.
    std::vector<ReachNodePtr> propagate_reachable_set(const std::vector<ReachNodePtr>& vec_nodes);

    std::vector<ReachNodePtr>
    compute_reachable_set_at_time_step(int const& time_step,
                                       std::vector<ReachNodePtr> const& vec_base_sets_propagated,
                                       std::vector<ReachPolygonPtr> const& drivable_area);
};

using ReachableSetPtr = shared_ptr<ReachableSet>;
}