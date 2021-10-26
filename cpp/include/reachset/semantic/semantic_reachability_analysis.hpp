#pragma once
#include "reachset/common/utility/shared_include.hpp"
#include "reachset/common/data_structure/reach_polygon.hpp"
#include "reachset/common/data_structure/reach_node.hpp"
#include "reachset/common/data_structure/configuration.hpp"
#include "reachset/semantic/data_structure/semantic_model.hpp"
#include "collision/collision_checker.h"
#include <omp.h>

using CollisionCheckerPtr = collision::CollisionCheckerPtr;

namespace reach {
/// Semantic reachability analysis of vehicles.
class SemanticReachabilityAnalysis {
private:
    ConfigurationPtr _config;
    CollisionCheckerPtr _collision_checker;
    SemanticModelPtr _semantic_model;
    ReachPolygonPtr _polygon_zero_state_lon;
    ReachPolygonPtr _polygon_zero_state_lat;

public:
    SemanticReachabilityAnalysis() = default;

    SemanticReachabilityAnalysis(ConfigurationPtr config, CollisionCheckerPtr collision_checker,
                                 SemanticModelPtr semantic_model);

    inline ConfigurationPtr config() const { return _config; };

    inline CollisionCheckerPtr collision_checker() const { return _collision_checker; }

    inline ReachPolygonPtr polygon_zero_state_lon() const { return _polygon_zero_state_lon; }

    inline ReachPolygonPtr polygon_zero_state_lat() const { return _polygon_zero_state_lat; }

    std::map<std::set<std::string>, std::vector<ReachPolygonPtr>> initial_drivable_area() const;

    std::set<std::string> obtain_propositions_for_rectangle(ReachPolygonPtr const& rectangle) const;

    std::map<std::set<std::string>, std::vector<ReachNodePtr>> initial_reachable_set() const;

    /// Initializes the zero-state polygons of the system.
    void initialize_zero_state_polygons();

    std::tuple<std::map<std::set<std::string>, std::vector<ReachPolygonPtr>>,
            std::map<std::set<std::string>, std::vector<ReachNodePtr>>> compute_drivable_area_at_time_step(
            int const& time_step,
            std::map<std::set<std::string>, std::vector<ReachNodePtr>> const& reachable_set_time_step_previous);

    /// Propagates the nodes of the reachable set from the last time step.
    std::vector<ReachNodePtr> propagate_reachable_set(const std::vector<ReachNodePtr>& vec_nodes);

    std::vector<ReachNodePtr> adapt_base_sets_to_regions(std::vector<ReachNodePtr> const& vec_base_sets) const;

    std::vector<ReachNodePtr>
    adapt_base_sets_to_vehicles(int const& time_step, std::vector<ReachNodePtr> const& vec_base_sets) const;

    static ReachNodePtr adapt_base_set_to_interval(
            ReachNodePtr const& base_set, PositionIntervalPtr const& interval, std::string const& direction);

    static std::vector<ReachNodePtr> discard_colliding_base_sets(std::vector<ReachNodePtr> const& vec_base_sets);

    std::map<std::set<std::string>, std::vector<ReachNodePtr>> compute_reachable_set_at_time_step(
            int const& time_step,
            std::map<std::set<std::string>, std::vector<ReachNodePtr>> & map_propositions_to_vec_base_sets_propagated,
            std::map<std::set<std::string>, std::vector<ReachPolygonPtr>> const& map_propositions_to_drivable_area);
};

using SemanticReachabilityAnalysisPtr = shared_ptr<SemanticReachabilityAnalysis>;
}