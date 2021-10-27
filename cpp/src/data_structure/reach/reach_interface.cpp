#include "reachset/data_structure/reach/reach_interface.hpp"
#include "reachset/data_structure/reach/reach_analysis.hpp"
#include "reachset/semantic/semantic_reachability_analysis.hpp"
#include "reachset/utility/shared_using.hpp"

#include "reachset/utility/miscellaneous.hpp"

using namespace reach;

reach::ReachableSetInterface::ReachableSetInterface(ContinuousReachabilityAnalysisPtr& reachability_analysis) {
    _mode = ReachabilityMode::CONTINUOUS;
    time_step_start = reachability_analysis->config()->planning().time_step_start;
    time_step_end = time_step_start + reachability_analysis->config()->planning().time_steps_computation;

    _reachability_analysis_continuous = reachability_analysis;
    time_step_start = reachability_analysis->config()->planning().time_step_start;

    _map_time_to_drivable_area_continuous[time_step_start] = reachability_analysis->initial_drivable_area();
    _map_time_to_reachable_set_continuous[time_step_start] = reachability_analysis->initial_reachable_set();
}

reach::ReachableSetInterface::ReachableSetInterface(SemanticReachabilityAnalysisPtr& reachability_analysis) {
    _mode = ReachabilityMode::SEMANTIC;
    time_step_start = reachability_analysis->config()->planning().time_step_start;
    time_step_end = time_step_start + reachability_analysis->config()->planning().time_steps_computation;

    _reachability_analysis_semantic = reachability_analysis;
    time_step_start = reachability_analysis->config()->planning().time_step_start;
    // iterate through propositions and append drivable area
    for (auto const&[set_propositions, vec_drivable_area]: reachability_analysis->initial_drivable_area()) {
        _map_time_to_drivable_area_semantic[time_step_start][set_propositions].insert(
                _map_time_to_drivable_area_semantic[time_step_start][set_propositions].end(),
                vec_drivable_area.begin(), vec_drivable_area.end());
    }

    // iterate through propositions and append reachable set
    for (auto const&[set_propositions, vec_reachable_set]: reachability_analysis->initial_reachable_set()) {
        _map_time_to_reachable_set_semantic[time_step_start][set_propositions].insert(
                _map_time_to_reachable_set_semantic[time_step_start][set_propositions].end(),
                vec_reachable_set.begin(), vec_reachable_set.end());
    }
}

ReachableSetInterface ReachableSetInterface::continuous(
        ConfigurationPtr const& config, CollisionCheckerPtr const& collision_checker) {
    auto reachability_analysis = make_shared<ContinuousReachabilityAnalysis>(config, collision_checker);
    return ReachableSetInterface(reachability_analysis);
}

ReachableSetInterface ReachableSetInterface::semantic(
        ConfigurationPtr const& config, CollisionCheckerPtr const& collision_checker,
        SemanticModelPtr const& semantic_model) {
    auto reachability_analysis = make_shared<SemanticReachabilityAnalysis>(config, collision_checker, semantic_model);
    return ReachableSetInterface(reachability_analysis);
}

void ReachableSetInterface::compute(int const& time_step_start, int time_step_end) {
    if (time_step_start == 0) {
        throw std::logic_error("<ReachableSetInterface> Time step should not start with 0.");
    }

    if (time_step_end == 0)
        time_step_end = config()->planning().time_steps_computation;

    cout << "Computing for time steps: [" << time_step_start << ", " << time_step_end << "]" << endl;
    for (int time_step = time_step_start; time_step < time_step_end + 1; ++time_step) {
        compute_at_time_step(time_step);
    }

    set_maps();
}

void ReachableSetInterface::compute_at_time_step(int const& time_step) {
    compute_drivable_area_at_time_step(time_step);
    compute_reachable_set_at_time_step(time_step);
}

void ReachableSetInterface::compute_drivable_area_at_time_step(int const& time_step) {
    if (_mode == ReachabilityMode::CONTINUOUS) {
        // prepare
        auto reachable_set_time_step_previous = _map_time_to_reachable_set_continuous[time_step - 1];
        // compute
        auto[drivable_area, vec_base_sets_propagated] =
        _reachability_analysis_continuous->compute_drivable_area_at_time_step(time_step,
                                                                              reachable_set_time_step_previous);
        // store
        _map_time_to_drivable_area_continuous[time_step] = drivable_area;
        _map_time_to_vec_base_sets_propagated_continuous[time_step] = vec_base_sets_propagated;
    } else if (_mode == ReachabilityMode::SEMANTIC) {
        // prepare
        auto reachable_set_time_step_previous = _map_time_to_reachable_set_semantic[time_step - 1];
        // compute
        auto[map_propositions_to_drivable_area, map_propositions_to_vec_base_sets_adapted] =
        _reachability_analysis_semantic->compute_drivable_area_at_time_step(time_step,
                                                                            reachable_set_time_step_previous);
        // store
        _map_time_to_drivable_area_semantic[time_step] = map_propositions_to_drivable_area;
        _map_time_to_vec_base_sets_propagated_semantic[time_step] = map_propositions_to_vec_base_sets_adapted;
    } else throw std::logic_error("<ReachableSetInterface> Invalid mode.");
}

void ReachableSetInterface::compute_reachable_set_at_time_step(int const& time_step) {
    if (_mode == ReachabilityMode::CONTINUOUS) {
        // prepare
        auto reachable_set_time_step_previous = _map_time_to_reachable_set_continuous[time_step - 1];
        auto vec_base_sets_propagated = _map_time_to_vec_base_sets_propagated_continuous[time_step];
        auto drivable_area = _map_time_to_drivable_area_continuous[time_step];
        // compute
        auto reachable_set = _reachability_analysis_continuous->compute_reachable_set_at_time_step(
                time_step, vec_base_sets_propagated, drivable_area);
        // store
        _map_time_to_reachable_set_continuous[time_step] = reachable_set;
    } else if (_mode == ReachabilityMode::SEMANTIC) {
        // prepare
        auto map_propositions_to_vec_base_sets_adapted = _map_time_to_vec_base_sets_propagated_semantic[time_step];
        auto map_propositions_to_drivable_area = _map_time_to_drivable_area_semantic[time_step];
        // compute
        auto reachable_set = _reachability_analysis_semantic->compute_reachable_set_at_time_step(
                time_step, map_propositions_to_vec_base_sets_adapted, map_propositions_to_drivable_area);
        // store
        _map_time_to_reachable_set_semantic[time_step] = reachable_set;
    } else throw std::logic_error("<ReachableSetInterface> Invalid mode.");
}

void ReachableSetInterface::set_maps() {
    if (_mode == ReachabilityMode::CONTINUOUS) {
        map_time_to_drivable_area = _map_time_to_drivable_area_continuous;
        map_time_to_reachable_set = _map_time_to_reachable_set_continuous;

    } else if (_mode == ReachabilityMode::SEMANTIC) {
        for (auto const&[time_step, map_propositions_to_drivable_areas]: _map_time_to_drivable_area_semantic) {
            for (auto const&[propositions, vec_drivable_areas]: map_propositions_to_drivable_areas) {
                map_time_to_drivable_area[time_step].insert(map_time_to_drivable_area[time_step].end(),
                                                            vec_drivable_areas.cbegin(),
                                                            vec_drivable_areas.cend());
            }
        }

        for (auto const&[time_step, map_propositions_to_reachable_sets]: _map_time_to_reachable_set_semantic) {
            for (auto const&[propositions, vec_reachable_sets]: map_propositions_to_reachable_sets) {
                map_time_to_reachable_set[time_step].insert(map_time_to_reachable_set[time_step].end(),
                                                            vec_reachable_sets.cbegin(),
                                                            vec_reachable_sets.cend());
            }
        }
    } else throw std::logic_error("<ReachableSetInterface> Invalid mode.");
}

void ReachableSetInterface::add_refined_node(int const& time_step, ReachNodePtr const& node) {
    map_time_to_reachable_set_refined[time_step].emplace_back(node);
}

void ReachableSetInterface::print_collision_checker_info() {
    print_collision_checker(_reachability_analysis_continuous->collision_checker());
}