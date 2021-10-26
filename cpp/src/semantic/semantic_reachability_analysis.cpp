#include "reachset/semantic/semantic_reachability_analysis.hpp"

#include <utility>
#include "reachset/common/utility/shared_using.hpp"
#include "reachset/common/utility/reach_operation.hpp"

using namespace reach;

SemanticReachabilityAnalysis::SemanticReachabilityAnalysis(
        ConfigurationPtr config, CollisionCheckerPtr collision_checker, SemanticModelPtr semantic_model) :
        _config(std::move(config)), _collision_checker(std::move(collision_checker)),
        _semantic_model(std::move(semantic_model)) {
    initialize_zero_state_polygons();
}

/// @note Computation of the reachable set of an LTI system requires the zero-state response and
/// zero-input response of the system.
void SemanticReachabilityAnalysis::initialize_zero_state_polygons() {
    _polygon_zero_state_lon = create_zero_state_polygon(_config->planning().dt,
                                                        _config->vehicle().ego.a_lon_min,
                                                        _config->vehicle().ego.a_lon_max);

    _polygon_zero_state_lat = create_zero_state_polygon(_config->planning().dt,
                                                        _config->vehicle().ego.a_lat_min,
                                                        _config->vehicle().ego.a_lat_max);
}

map<set<string>, vector<ReachPolygonPtr>> SemanticReachabilityAnalysis::initial_drivable_area() const {
    map<set<string>, vector<ReachPolygonPtr>> map_proposition_to_drivable_areas;

    auto tuple_vertices = generate_tuple_vertices_position_rectangle_initial(_config);
    auto drivable_initial = std::apply(ReachPolygon::from_rectangle_coordinates, tuple_vertices);
    auto propositions = obtain_propositions_for_rectangle(drivable_initial);
    map_proposition_to_drivable_areas[propositions].emplace_back(drivable_initial);

    return map_proposition_to_drivable_areas;
}

set<string> SemanticReachabilityAnalysis::obtain_propositions_for_rectangle(ReachPolygonPtr const& rectangle) const {
    set<string> set_propositions{};
    /// find the intersecting region and update propositions accordingly
    for (auto const& region: _semantic_model->vec_regions) {
        if (not region->excludes(rectangle) and region->polygon_cvln->intersects(rectangle)) {
            set_propositions.insert(region->set_propositions.begin(), region->set_propositions.end());
            break;
        }
    }

    /// find the intersecting longitudinal interval and update propositions accordingly
    for (auto const& interval_lon: _semantic_model->map_time_step_to_position_intervals[0]["lon"]) {
        if (interval_lon->intersects(rectangle->p_lon_min(), rectangle->p_lon_max())) {
            set_propositions.insert(interval_lon->set_propositions.begin(), interval_lon->set_propositions.end());
            break;
        }
    }

    /// find the intersecting lateral interval and update propositions accordingly
    for (auto const& interval_lat: _semantic_model->map_time_step_to_position_intervals[0]["lat"]) {
        if (interval_lat->intersects(rectangle->p_lat_min(), rectangle->p_lat_max())) {
            set_propositions.insert(interval_lat->set_propositions.begin(), interval_lat->set_propositions.end());
            break;
        }
    }

    return set_propositions;
}

map<set<string>, vector<ReachNodePtr>> SemanticReachabilityAnalysis::initial_reachable_set() const {
    map<set<string>, vector<ReachNodePtr>> map_proposition_to_reachable_sets;
    /// retrieve propositions as in initial drivable area
    auto tuple_vertices = generate_tuple_vertices_position_rectangle_initial(_config);
    auto drivable_initial = std::apply(ReachPolygon::from_rectangle_coordinates, tuple_vertices);
    auto propositions = obtain_propositions_for_rectangle(drivable_initial);

    /// construct initial node
    auto[tuple_vertices_polygon_lon, tuple_vertices_polygon_lat] = generate_tuples_vertices_polygons_initial(_config);
    auto polygon_lon = std::apply(ReachPolygon::from_rectangle_coordinates, tuple_vertices_polygon_lon);
    auto polygon_lat = std::apply(ReachPolygon::from_rectangle_coordinates, tuple_vertices_polygon_lat);
    auto node_initial = make_shared<ReachNode>(_config->planning().time_step_start,
                                               polygon_lon, polygon_lat, propositions);
    map_proposition_to_reachable_sets[propositions].emplace_back(node_initial);

    return map_proposition_to_reachable_sets;
}

/// *Steps*:
/// For nodes within each set of propositions, perform the following:
/// 1. Propagate each node of the reachable set from the last time step. This forms a list of propagated base sets.
/// 2.1 Adapt to lanelet regions
/// 2.2 Adapt to position intervals
/// 2.3 Discard colliding base sets
/// 3. Project the base sets onto the position domain.
/// 4. Merge and repartition these rectangles to reduce computation load.
/// 5. Check for collision and split the repartitioned rectangles into collision-free rectangles.
/// 6. Merge and repartition the collision-free rectangles again to reduce number of nodes.
tuple<map<set<string>, vector<ReachPolygonPtr>>, map<set<string>, vector<ReachNodePtr>>>
SemanticReachabilityAnalysis::compute_drivable_area_at_time_step(
        int const& time_step, map<set<string>, vector<ReachNodePtr>> const& reachable_set_time_step_previous) {

    if (reachable_set_time_step_previous.empty())
        return {{},
                {}};

    map<set<string>, vector<ReachNodePtr>> map_propositions_to_vec_base_sets_adapted{};
    map<set<string>, vector<ReachPolygonPtr>> map_propositions_to_drivable_area;

    for (auto const&[propositions, vec_nodes]: reachable_set_time_step_previous) {
        // Step 1
        auto vec_base_sets_propagated = propagate_reachable_set(vec_nodes);

        // Step 2.1
        auto vec_base_sets_adapted_to_regions = adapt_base_sets_to_regions(vec_base_sets_propagated);

        // Step 2.2
        auto vec_base_sets_adapted_to_vehicles = adapt_base_sets_to_vehicles(time_step,
                                                                             vec_base_sets_adapted_to_regions);

        // Step 2.3
        auto vec_base_sets_collision_free = discard_colliding_base_sets(vec_base_sets_adapted_to_vehicles);

        for (auto const& base_set: vec_base_sets_collision_free) {
            map_propositions_to_vec_base_sets_adapted[base_set->set_propositions].emplace_back(base_set);
        }
    }

    // iterate through vectors of base sets with different propositions and process them individually
    for (auto const&[propositions, vec_base_sets]: map_propositions_to_vec_base_sets_adapted) {
        // Step 3
        auto vec_rectangles_projected = project_base_sets_to_position_domain(vec_base_sets);

        // Step 4
        auto vec_rectangles_repartitioned = create_repartitioned_rectangles(vec_rectangles_projected,
                                                                            _config->reachable_set().size_grid);
        // Step 5
        auto vec_rectangles_collision_free = check_collision_and_split_rectangles(
                std::floor(time_step * _config->planning().dt * 10),
                _collision_checker,
                vec_rectangles_repartitioned,
                _config->reachable_set().radius_terminal_split,
                _config->reachable_set().num_threads);

        // Step 6
        map_propositions_to_drivable_area[propositions] =
                create_repartitioned_rectangles(vec_rectangles_collision_free, _config->reachable_set().size_grid_2nd);
    }

    return {map_propositions_to_drivable_area, map_propositions_to_vec_base_sets_adapted};
}

vector<ReachNodePtr>
SemanticReachabilityAnalysis::propagate_reachable_set(vector<ReachNodePtr> const& vec_nodes) {
    vector<ReachNodePtr> vec_base_sets_propagated;
    vec_base_sets_propagated.reserve(vec_nodes.size());

#pragma omp parallel num_threads(_config->reachable_set().num_threads)
    {
        vector<ReachNodePtr> vec_base_sets_propagated_thread;
        vec_base_sets_propagated_thread.reserve(vec_nodes.size());

#pragma omp for nowait
        for (auto const& node: vec_nodes) {
            try {
                auto polygon_lon_propagated = propagate_polygon(node->polygon_lon,
                                                                _polygon_zero_state_lon,
                                                                _config->planning().dt,
                                                                _config->vehicle().ego.v_lon_min,
                                                                _config->vehicle().ego.v_lon_max);

                auto polygon_lat_propagated = propagate_polygon(node->polygon_lat,
                                                                _polygon_zero_state_lat,
                                                                _config->planning().dt,
                                                                _config->vehicle().ego.v_lat_min,
                                                                _config->vehicle().ego.v_lat_max);

                auto base_set_propagated = make_shared<ReachNode>(node->time_step,
                                                                  polygon_lon_propagated, polygon_lat_propagated);
                base_set_propagated->vec_nodes_source.emplace_back(node);
                vec_base_sets_propagated_thread.emplace_back(base_set_propagated);
            }
            catch (std::exception& e) {
                continue;
            }
        }
#pragma omp critical
        vec_base_sets_propagated.insert(vec_base_sets_propagated.end(),
                                        std::make_move_iterator(vec_base_sets_propagated_thread.begin()),
                                        std::make_move_iterator(vec_base_sets_propagated_thread.end()));
    }
    return vec_base_sets_propagated;
}

vector<ReachNodePtr>
SemanticReachabilityAnalysis::adapt_base_sets_to_regions(vector<ReachNodePtr> const& vec_base_sets) const {
    vector<ReachNodePtr> vec_base_sets_adapted{};

    if (vec_base_sets.empty()) return vec_base_sets_adapted;

    for (auto const& region: _semantic_model->vec_regions) {
        for (auto const& base_set: vec_base_sets) {
            auto rectangle = base_set->position_rectangle();
            if (region->excludes(rectangle))
                continue;

            // there is a possibility of intersection
            auto vec_polygon_intersection = region->polygon_cvln->intersection(rectangle);
            if (vec_polygon_intersection.empty()) continue;

            // over-approximate by restoring to axis-aligned rectangles
            auto[p_lon_min, p_lat_min, p_lon_max, p_lat_max] =
            obtain_extremum_coordinates_of_polygons(vec_polygon_intersection);

            // clone the base set and adapt it in the position domain and update the propositions
            auto base_set_adapted = base_set->clone();
            base_set_adapted->intersect_in_position_domain(p_lon_min, p_lat_min, p_lon_max, p_lat_max);
            base_set_adapted->set_propositions.insert(region->set_propositions.begin(), region->set_propositions.end());

            vec_base_sets_adapted.emplace_back(base_set_adapted);
        }
    }

    return vec_base_sets_adapted;
}

vector<ReachNodePtr>
SemanticReachabilityAnalysis::adapt_base_sets_to_vehicles(int const& time_step,
                                                          vector<ReachNodePtr> const& vec_base_sets) const {
    vector<ReachNodePtr> vec_base_sets_adapted{};
    vector<ReachNodePtr> vec_base_sets_adapted_lon{};

    if (vec_base_sets.empty()) return vec_base_sets_adapted;

    auto vec_intervals_lon = _semantic_model->map_time_step_to_position_intervals[time_step]["lon"];
    auto vec_intervals_lat = _semantic_model->map_time_step_to_position_intervals[time_step]["lat"];

    for (auto const& base_set: vec_base_sets) {
        for (auto const& interval_lon: vec_intervals_lon) {
            if (interval_lon->intersects(base_set->p_lon_min(), base_set->p_lon_max())) {
                auto base_set_adapted = adapt_base_set_to_interval(base_set, interval_lon, "lon");
                if (base_set_adapted) {
                    vec_base_sets_adapted_lon.emplace_back(base_set_adapted);
                }
            }
            //else if(interval_lon->p_min > base_set->polygon_lon->p_max()) break;
        }
    }

    for (auto const& base_set: vec_base_sets_adapted_lon) {
        for (auto const& interval_lat: vec_intervals_lat) {
            if (interval_lat->intersects(base_set->p_lat_min(), base_set->p_lat_max())) {
                auto base_set_adapted = adapt_base_set_to_interval(base_set, interval_lat, "lat");
                if (base_set_adapted) {
                    vec_base_sets_adapted.emplace_back(base_set_adapted);
                }
            }
            //else if(interval_lat->p_min > base_set->polygon_lat->p_max()) break;
        }
    }

    return vec_base_sets_adapted;
}

ReachNodePtr SemanticReachabilityAnalysis::adapt_base_set_to_interval(
        ReachNodePtr const& base_set, PositionIntervalPtr const& interval, string const& direction) {
    auto base_set_adapted = base_set->clone();
    base_set_adapted->set_propositions.insert(interval->set_propositions.cbegin(), interval->set_propositions.cend());

    try {
        if (direction == "lon") {
            base_set_adapted->polygon_lon->intersect_halfspace(1, 0, interval->p_max);
            base_set_adapted->polygon_lon->intersect_halfspace(-1, 0, -interval->p_min);
        } else if (direction == "lat") {
            base_set_adapted->polygon_lat->intersect_halfspace(1, 0, interval->p_max);
            base_set_adapted->polygon_lat->intersect_halfspace(-1, 0, -interval->p_min);
        } else throw std::logic_error("<Interval> Given direction is not valid.");
    }
    catch (std::exception& e) {
        base_set_adapted = nullptr;
    }

    // check the validity of the adapted polygons
    if (not base_set_adapted or not(base_set_adapted->polygon_lon->valid() and base_set_adapted->polygon_lat->valid()))
        base_set_adapted = nullptr;

    return base_set_adapted;
}

vector<ReachNodePtr>
SemanticReachabilityAnalysis::discard_colliding_base_sets(vector<ReachNodePtr> const& vec_base_sets) {
    vector<ReachNodePtr> vec_base_sets_collision_free{};

    for (auto const& base_set: vec_base_sets) {
        bool colliding = false;
        auto& set_propositions = base_set->set_propositions;
        for (auto const& proposition: set_propositions) {
            if (ba::contains(proposition, "AlignedWith_")) {
                vector<string> result_split;
                ba::split(result_split, proposition, boost::is_any_of("_"));
                auto id_vehicle = result_split[1];

                if (set_propositions.find("Beside_" + id_vehicle) != set_propositions.end()) colliding = true;
            }
        }

        if (colliding) continue;
        vec_base_sets_collision_free.emplace_back(base_set);
    }

    return vec_base_sets_collision_free;
}

//vector<ReachNodePtr> SemanticReachabilityAnalysis::compute_reachable_set_at_time_step(
//        int const& time_step,
//        vector<ReachNodePtr> const& vec_base_sets_propagated,
//        vector<ReachPolygonPtr> const& drivable_area) {
//
//}

/// *Steps*:
/// 1. create a list of new base sets cut down with rectangles of the drivable area.
/// 2. create the list of nodes of the new reachable set.
map<set<string>, vector<ReachNodePtr>> SemanticReachabilityAnalysis::compute_reachable_set_at_time_step(
        int const& time_step,
        map<set<string>, vector<ReachNodePtr>>& map_propositions_to_vec_base_sets_propagated,
        map<set<string>, vector<ReachPolygonPtr>> const& map_propositions_to_drivable_area) {

    map<set<string>, vector<ReachNodePtr>> map_propositions_to_vec_reachable_sets;
    if (map_propositions_to_drivable_area.empty()) {
        return {};
    }

    for (auto const&[propositions, drivable_area]: map_propositions_to_drivable_area) {
        auto vec_base_sets_propagated = map_propositions_to_vec_base_sets_propagated[propositions];
        // Step 1
        auto vec_base_sets_adapted = adapt_base_sets_to_drivable_area(drivable_area, vec_base_sets_propagated,
                                                                      _config->reachable_set().num_threads);
        // Step 2
        auto reachable_set = create_reachable_set_nodes_semantic(time_step, vec_base_sets_adapted, propositions);
        map_propositions_to_vec_reachable_sets[propositions] = reachable_set;
    }

    return map_propositions_to_vec_reachable_sets;
}

