#pragma once

#include "reachset/continuous/continuous_reachability_analysis.hpp"
#include "reachset/semantic/semantic_reachability_analysis.hpp"
#include "reachset/common/data_structure/configuration.hpp"
#include "reachset/semantic/data_structure/semantic_model.hpp"
#include "collision/collision_checker.h"

using CollisionCheckerPtr = collision::CollisionCheckerPtr;

namespace reach {
/// Possible states of nodes (segments).
enum class ReachabilityMode {
    /// continuous reachability analysis
    CONTINUOUS,
    /// semantic reachability analysis
    SEMANTIC
};

/// Main interface to compute the reachable sets.
class ReachableSetInterface {
private:
    ReachabilityMode _mode;
    // continuous
    ContinuousReachabilityAnalysisPtr _reachability_analysis_continuous;
    std::map<int, std::vector<ReachPolygonPtr>> _map_time_to_drivable_area_continuous;
    std::map<int, std::vector<ReachNodePtr>> _map_time_to_vec_base_sets_propagated_continuous;
    std::map<int, std::vector<ReachNodePtr>> _map_time_to_reachable_set_continuous;
    // semantic
    SemanticReachabilityAnalysisPtr _reachability_analysis_semantic;
    std::map<int, std::map<std::set<std::string>, std::vector<ReachPolygonPtr>>> _map_time_to_drivable_area_semantic;
    std::map<int, std::map<std::set<std::string>, std::vector<ReachNodePtr>>> _map_time_to_vec_base_sets_propagated_semantic;
    std::map<int, std::map<std::set<std::string>, std::vector<ReachNodePtr>>> _map_time_to_reachable_set_semantic;

    /// Computes drivable area and reachable set at the specified time step.
    void compute_at_time_step(int const& time_step);

    /// Computes drivable area at the specified time step.
    /// Drivable area is computed based on reachable set of the last time step.
    void compute_drivable_area_at_time_step(int const& time_step);

    /// Computes reachable set at the specified time step.
    /// Reachable set is computed based on the drivable area and the list of propagated nodes. It is also based on
    /// the reachable set of the last time step.
    void compute_reachable_set_at_time_step(int const& time_step);

    /// Sets internal maps from time steps to drivable areas/reachable sets.
    void set_maps();

public:
    explicit ReachableSetInterface(ContinuousReachabilityAnalysisPtr& reachability_analysis);

    explicit ReachableSetInterface(SemanticReachabilityAnalysisPtr& reachability_analysis);

    /// Instantiates a reachable set interface with continuous reachability analysis.
    static ReachableSetInterface
    continuous(ConfigurationPtr const& config, CollisionCheckerPtr const& collision_checker);

    /// Instantiates a reachable set interface with semantic reachability analysis
    static ReachableSetInterface semantic(ConfigurationPtr const& config, CollisionCheckerPtr const& collision_checker,
                                          SemanticModelPtr const& semantic_model);

    int time_step_start{};
    int time_step_end{};
    std::map<int, std::vector<ReachPolygonPtr>> map_time_to_drivable_area;
    std::map<int, std::vector<ReachNodePtr>> map_time_to_reachable_set;
    std::map<int, std::vector<ReachNodePtr>> map_time_to_reachable_set_refined;

    inline ConfigurationPtr config() const {
        if (_mode == ReachabilityMode::CONTINUOUS) return _reachability_analysis_continuous->config();
        else if (_mode == ReachabilityMode::SEMANTIC) return _reachability_analysis_semantic->config();
        else throw std::logic_error("<ReachableSetInterface> Invalid mode.");
    }

    inline std::vector<ReachPolygonPtr> drivable_area_at_time_step(int const& time_step) {
        if (time_step_start <= time_step <= time_step_end + 1) {
            return map_time_to_drivable_area[time_step];
        } else throw std::logic_error("<ReachableSetInterface> Invalid time step.");
    }

    inline std::vector<ReachNodePtr> reachable_set_at_time_step(int const& time_step) {
        if (time_step_start <= time_step <= time_step_end + 1) {
            return map_time_to_reachable_set[time_step];
        } else throw std::logic_error("<ReachableSetInterface> Invalid time step.");
    }

    inline std::vector<ReachNodePtr> refined_reachable_set_at_time_step(int const& time_step) {
        if (time_step_start <= time_step <= time_step_end + 1) {
            return map_time_to_reachable_set_refined[time_step];
        } else throw std::logic_error("<ReachableSetInterface> Invalid time step.");
    }


    // Computes drivable area and reachable sets for the specified time range.
    void compute(int const& time_step_start = 1, int time_step_end = 0);

    void add_refined_node(int const& time_step, ReachNodePtr const& node);

    void print_collision_checker_info();
};

using ReachableSetInterfacePtr = std::shared_ptr<ReachableSetInterface>;
}