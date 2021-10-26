#pragma once

#include "reachset/common/utility/shared_include.hpp"
#include "reachset/common/data_structure/reach_polygon.hpp"
#include "reachset/common/data_structure/reach_node.hpp"
#include "reachset/common/data_structure/configuration.hpp"

namespace reach {
class ReachabilityAnalysis {
public:
    virtual ConfigurationPtr config() const = 0;

    /// Drivable area at the initial time step.
    virtual std::vector<ReachPolygonPtr> initial_drivable_area() const = 0;

    /// Reachable set at the initial time step.
    virtual std::vector<ReachNodePtr> initial_reachable_set() const = 0;

    /// Computes drivable area at the specified time step.
    virtual std::tuple<std::vector<ReachPolygonPtr>, std::vector<ReachBaseSetPtr>>
    compute_drivable_area_at_time_step(int const& time_step,
                                       std::vector<ReachNodePtr> const& reachable_set_time_step_previous) = 0;

    /// Computes reachable set at the specified time step.
    virtual std::vector<ReachNodePtr>
    compute_reachable_set_at_time_step(int const& time_step,
                                       std::vector<ReachNodePtr> const& reachable_set_time_step_previous,
                                       std::vector<ReachBaseSetPtr> const& vec_base_sets_propagated,
                                       std::vector<ReachPolygonPtr> const& drivable_area) = 0;
};

}