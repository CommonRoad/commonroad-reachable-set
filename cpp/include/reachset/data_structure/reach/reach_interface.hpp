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
    /// Sets internal maps from time steps to drivable areas/reachable sets.
    //void set_maps();

public:
    ReachableSetInterface(ConfigurationPtr const& config, CollisionCheckerPtr const& collision_checker);

    ConfigurationPtr config{nullptr};

    int time_step_start{};
    int time_step_end{};

    inline std::map<int, std::vector<ReachPolygonPtr>> drivable_area() const {
        return _reachable_set->map_time_to_drivable_area;
    }

    inline std::map<int, std::vector<ReachNodePtr>> reachable_set() const {
        return _reachable_set->map_time_to_reachable_set;
    }

    inline std::vector<ReachPolygonPtr> drivable_area_at_time_step(int const& time_step) const {
        if (!_reachable_set_computed and time_step != 0) {
            cout << "Reachable set is not computed, retrieving drivable area failed." << endl;
            return {};
        } else {
            return _reachable_set->drivable_area_at_time_step(time_step);
        }
    }

    inline std::vector<ReachNodePtr> reachable_set_at_time_step(int const& time_step) const {
        if (!_reachable_set_computed and time_step != 0) {
            cout << "Reachable set is not computed, retrieving reachable set failed." << endl;
            return {};
        } else {
            return _reachable_set->reachable_set_at_time_step(time_step);
        }
    }

    // Computes reachable sets for the specified time range.
    void compute_reachable_sets(int const& step_start = 1, int step_end = 0);

    //// Adds a node to the pruned reachable sets.
    //void add_pruned_node(int const& time_step, ReachNodePtr const& node);
};

using ReachableSetInterfacePtr = std::shared_ptr<ReachableSetInterface>;
}