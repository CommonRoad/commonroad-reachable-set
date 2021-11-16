#include "reachset/data_structure/reach/reach_interface.hpp"
#include "reachset/data_structure/reach/reach_analysis.hpp"
#include "reachset/utility/shared_using.hpp"

#include "reachset/utility/miscellaneous.hpp"

using namespace reach;

reach::ReachableSetInterface::ReachableSetInterface(
        ConfigurationPtr const& config, CollisionCheckerPtr const& collision_checker) {
    reachability_analysis = make_shared<ReachabilityAnalysis>(config, collision_checker);

    time_step_start = reachability_analysis->config()->planning().time_step_start;
    time_step_end = time_step_start + reachability_analysis->config()->planning().time_steps_computation;
    time_step_start = reachability_analysis->config()->planning().time_step_start;

    _map_time_to_drivable_area[time_step_start] = reachability_analysis->initial_drivable_area();
    _map_time_to_reachable_set[time_step_start] = reachability_analysis->initial_reachable_set();
}

void ReachableSetInterface::compute_reachable_sets(int const& step_start, int step_end) {
    if (step_start == 0) throw std::logic_error("<ReachableSetInterface> Time step should not start with 0.");
    if (step_end == 0) step_end = config()->planning().time_steps_computation;

    for (int time_step = step_start; time_step < step_end + 1; ++time_step) {
        compute_at_time_step(time_step);
    }
}

void ReachableSetInterface::compute_at_time_step(int const& time_step) {
    compute_drivable_area_at_time_step(time_step);
    compute_reachable_set_at_time_step(time_step);
    set_maps();
}

void ReachableSetInterface::compute_drivable_area_at_time_step(int const& time_step) {
    // prepare
    auto reachable_set_time_step_previous = _map_time_to_reachable_set[time_step - 1];
    // compute
    auto[drivable_area, vec_base_sets_propagated] =
    reachability_analysis->compute_drivable_area_at_time_step(time_step,
                                                              reachable_set_time_step_previous);
    // store
    _map_time_to_drivable_area[time_step] = drivable_area;
    _map_time_to_vec_base_sets_propagated[time_step] = vec_base_sets_propagated;
}

void ReachableSetInterface::compute_reachable_set_at_time_step(int const& time_step) {
    // prepare
    auto reachable_set_time_step_previous = _map_time_to_reachable_set[time_step - 1];
    auto vec_base_sets_propagated = _map_time_to_vec_base_sets_propagated[time_step];
    auto drivable_area = _map_time_to_drivable_area[time_step];
    // compute
    auto reachable_set = reachability_analysis->compute_reachable_set_at_time_step(
            time_step, vec_base_sets_propagated, drivable_area);
    // store
    _map_time_to_reachable_set[time_step] = reachable_set;
}

void ReachableSetInterface::set_maps() {
    map_time_to_drivable_area = _map_time_to_drivable_area;
    map_time_to_reachable_set = _map_time_to_reachable_set;
}

//void ReachableSetInterface::add_pruned_node(int const& time_step, ReachNodePtr const& node) {
//    map_time_to_reachable_set_pruned[time_step].emplace_back(node);
//}