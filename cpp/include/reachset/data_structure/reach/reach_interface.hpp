#pragma once

#include "reach_set.hpp"
#include "reachset/data_structure/configuration.hpp"
#include "collision/collision_checker.h"

using CollisionCheckerPtr = collision::CollisionCheckerPtr;

namespace reach {
/// Main interface for reachable set computation.
class ReachableSetInterface {
private:
    ReachableSetPtr _reachable_set{nullptr};
    bool _reachable_set_computed{false};
    //std::map<int, std::vector<ReachPolygonPtr>> _map_time_to_drivable_area;
    //std::map<int, std::vector<ReachNodePtr>> _map_time_to_vec_base_sets_propagated;
    //std::map<int, std::vector<ReachNodePtr>> _map_time_to_reachable_set;

    /// Computes drivable area at the specified time step.
    /// Drivable area is computed based on reachable set of the last time step.
    //void compute_drivable_area_at_time_step(int const& time_step);

    /// Computes reachable set at the specified time step.
    /// Reachable set is computed based on the drivable area and the list of propagated nodes. It is also based on
    /// the reachable set of the last time step.
    //void compute_reachable_set_at_time_step(int const& time_step);

    /// Sets internal maps from time steps to drivable areas/reachable sets.
    //void set_maps();

public:
    ReachableSetInterface(ConfigurationPtr const& config, CollisionCheckerPtr const& collision_checker);

    ConfigurationPtr config{nullptr};

    int time_step_start{};
    int time_step_end{};
    //std::map<int, std::vector<ReachPolygonPtr>> map_time_to_drivable_area;
    //std::map<int, std::vector<ReachNodePtr>> map_time_to_reachable_set;
    //std::map<int, std::vector<ReachNodePtr>> map_time_to_reachable_set_pruned;

    inline std::map<int, std::vector<ReachPolygonPtr>> drivable_area() const { return _reachable_set->map_time_to_drivable_area; }
    inline std::map<int, std::vector<ReachNodePtr>> reachable_set() const { return _reachable_set->map_time_to_reachable_set; }
    inline std::map<int, std::vector<ReachNodePtr>> pruned_reachable_set() const { return _reachable_set->map_time_to_reachable_set_pruned; }

    inline std::vector<ReachPolygonPtr> drivable_area_at_time_step(int const& time_step) const {
        if (!_reachable_set_computed and time_step != 0) {
            cout << "Reachable set is not computed, retrieving drivable area failed." << endl;
        } else if (time_step_start <= time_step <= time_step_end + 1) {
            return _reachable_set->drivable_area_at_time_step(time_step);
        } else throw std::logic_error("Invalid time step.");
    }

    inline std::vector<ReachNodePtr> reachable_set_at_time_step(int const& time_step) const {
        if (!_reachable_set_computed and time_step != 0) {
            cout << "Reachable set is not computed, retrieving reachable set failed." << endl;
        } else if (time_step_start <= time_step <= time_step_end + 1) {
            return map_time_to_reachable_set[time_step];
        } else throw std::logic_error("Invalid time step.");
    }

    inline std::vector<ReachNodePtr> pruned_reachable_set_at_time_step(int const& time_step) const {
        if (!_reachable_set_computed and time_step != 0) {
            cout << "Reachable set is not computed, retrieving pruned reachable set failed." << endl;
        } else if (time_step_start <= time_step <= time_step_end + 1) {
            return map_time_to_reachable_set_pruned[time_step];
        } else throw std::logic_error("Invalid time step.");
    }

    // Computes reachable sets for the specified time range.
    void compute_reachable_sets(int const& step_start = 1, int step_end = 0);

    /// Computes drivable area and reachable set at the specified time step.
    //void compute_at_time_step(int const& time_step);

    //// Adds a node to the pruned reachable sets.
    //void add_pruned_node(int const& time_step, ReachNodePtr const& node);
};

using ReachableSetInterfacePtr = std::shared_ptr<ReachableSetInterface>;
}