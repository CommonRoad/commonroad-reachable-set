#pragma once

#include "reachset/utility/shared_include.hpp"
#include "reachset/common/data_structure/reach_polygon.hpp"
#include "reachset/common/data_structure/reach_node.hpp"
#include "reachset/common/data_structure/configuration.hpp"
#include "collision/collision_checker.h"
#include <omp.h>

using CollisionCheckerPtr = collision::CollisionCheckerPtr;

namespace reach {
/// Continuous reachability analysis of vehicles.
class ContinuousReachabilityAnalysis {
private:
    ConfigurationPtr _config;
    CollisionCheckerPtr _collision_checker;
    ReachPolygonPtr _polygon_zero_state_lon;
    ReachPolygonPtr _polygon_zero_state_lat;

public:
    ContinuousReachabilityAnalysis() = default;

    explicit ContinuousReachabilityAnalysis(ConfigurationPtr config);

    ContinuousReachabilityAnalysis(ConfigurationPtr config, CollisionCheckerPtr collision_checker);

    inline ConfigurationPtr config() const { return _config; };

    inline CollisionCheckerPtr collision_checker() const { return _collision_checker; }

    inline ReachPolygonPtr polygon_zero_state_lon() const { return _polygon_zero_state_lon; }

    inline ReachPolygonPtr polygon_zero_state_lat() const { return _polygon_zero_state_lat; }

    std::vector<ReachPolygonPtr> initial_drivable_area() const;

    std::vector<ReachNodePtr> initial_reachable_set() const;

    /// Initializes the zero-state polygons of the system.
    void initialize_zero_state_polygons();

    std::tuple<std::vector<ReachPolygonPtr>, std::vector<ReachNodePtr>>
    compute_drivable_area_at_time_step(int const& time_step,
                                       std::vector<ReachNodePtr> const& reachable_set_time_step_previous);

    /// Propagates the nodes of the reachable set from the last time step.
    std::vector<ReachNodePtr> propagate_reachable_set(const std::vector<ReachNodePtr>& vec_nodes);

    std::vector<ReachNodePtr>
    compute_reachable_set_at_time_step(int const& time_step,
                                       std::vector<ReachNodePtr> const& vec_base_sets_propagated,
                                       std::vector<ReachPolygonPtr> const& drivable_area);
};

using ContinuousReachabilityAnalysisPtr = shared_ptr<ContinuousReachabilityAnalysis>;
}