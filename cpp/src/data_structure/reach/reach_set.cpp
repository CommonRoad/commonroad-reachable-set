#include "reachset/data_structure/reach/reach_set.hpp"

#include <utility>
#include "reachset/utility/shared_using.hpp"
#include "reachset/utility/reach_operation.hpp"

using namespace reach;

ReachableSet::ReachableSet(ConfigurationPtr config) : config(std::move(config)) {
    _initialize();
}

ReachableSet::ReachableSet(ConfigurationPtr config, CollisionCheckerPtr collision_checker) :
        config(std::move(config)), collision_checker(std::move(collision_checker)) {
    _initialize();
}

void ReachableSet::_initialize() {
    time_step_start = config->planning().time_step_start;
    time_step_end = time_step_start + config->planning().time_steps_computation;
    map_time_to_drivable_area[time_step_start] = _construct_initial_drivable_area();
    map_time_to_reachable_set[time_step_start] = _construct_initial_reachable_set();

    _initialize_zero_state_polygons();
}

vector<ReachPolygon2Ptr> ReachableSet::_construct_initial_drivable_area() const {
    vector<ReachPolygon2Ptr> vec_polygon;

    auto tuple_vertices = generate_tuple_vertices_position_rectangle_initial(config);
    vec_polygon.emplace_back(make_shared<ReachPolygon2>(std::get<0>(tuple_vertices),
                                                        std::get<1>(tuple_vertices),
                                                        std::get<2>(tuple_vertices),
                                                        std::get<3>(tuple_vertices)));
    return vec_polygon;
}

std::vector<ReachNodePtr> ReachableSet::_construct_initial_reachable_set() const {
    vector<ReachNodePtr> vec_node;

    auto[tuple_vertices_polygon_lon, tuple_vertices_polygon_lat] = generate_tuples_vertices_polygons_initial(config);
    auto polygon_lon = make_shared<ReachPolygon2>(tuple_vertices_polygon_lon);
    auto polygon_lat = make_shared<ReachPolygon2>(tuple_vertices_polygon_lat);
    vec_node.emplace_back(make_shared<ReachNode>(config->planning().time_step_start, polygon_lon, polygon_lat));

    return vec_node;
}

/// @note Computation of the reachable set of an LTI system requires the zero-state response and
/// the zero-input response of the system.
void ReachableSet::_initialize_zero_state_polygons() {
    polygon_zero_state_lon = create_zero_state_polygon(config->planning().dt,
                                                       config->vehicle().ego.a_lon_min,
                                                       config->vehicle().ego.a_lon_max);

    polygon_zero_state_lat = create_zero_state_polygon(config->planning().dt,
                                                       config->vehicle().ego.a_lat_min,
                                                       config->vehicle().ego.a_lat_max);
}

void ReachableSet::compute(int step_start, int step_end) {
    if (step_start == 0) step_start = time_step_start + 1;
    if (step_end == 0) step_end = time_step_end;

    for (auto time_step = step_start; time_step < step_end + 1; time_step++) {
        _compute_drivable_area_at_time_step(time_step);
        _compute_reachable_set_at_time_step(time_step);
        _vec_time_steps_computed.emplace_back(time_step);
    }

    if (_prune_reachable_set) {
        _prune_nodes_not_reaching_final_time_step();
    }
}

/// *Steps*:
/// 1. Propagate each node of the reachable set from the last time step. This forms a list of propagated base sets.
/// 2. Project the base sets onto the position domain.
/// 3. Merge and repartition these rectangles to reduce computation load.
/// 4. Check for collision and split the repartitioned rectangles into collision-free rectangles.
/// 5. Merge and repartition the collision-free rectangles again to reduce number of nodes.
void ReachableSet::_compute_drivable_area_at_time_step(int const& time_step) {
    auto reachable_set_previous = map_time_to_reachable_set[time_step - 1];
    if (reachable_set_previous.empty()) {
        return;
    }
    
    auto vec_base_sets_propagated = _propagate_reachable_set(reachable_set_previous);

    auto vec_rectangles_projected = project_base_sets_to_position_domain(vec_base_sets_propagated);

    auto vec_rectangles_collision_free = check_collision_and_split_rectangles(
            std::floor(time_step * config->planning().dt * 10),
            collision_checker,
            vec_rectangles_projected,
            config->reachable_set().radius_terminal_split,
            config->reachable_set().num_threads);

    auto vec_rectangles_repartitioned = create_repartitioned_rectangles(
            vec_rectangles_collision_free,
            config->reachable_set().size_grid);

    map_time_to_drivable_area[time_step] = vec_rectangles_repartitioned;
    map_time_to_base_set_propagated[time_step] = vec_base_sets_propagated;
}

vector<ReachNodePtr> ReachableSet::_propagate_reachable_set(vector<ReachNodePtr> const& vec_nodes) {
    vector<ReachNodePtr> vec_base_sets_propagated;
    vec_base_sets_propagated.reserve(vec_nodes.size());

#pragma omp parallel num_threads(config->reachable_set().num_threads) \
default(none) shared(vec_nodes, vec_base_sets_propagated)
    {
        vector<ReachNodePtr> vec_base_sets_propagated_thread;
        vec_base_sets_propagated_thread.reserve(vec_nodes.size());

#pragma omp for nowait
        for (auto const& node: vec_nodes) {
            try {
                auto polygon_lon_propagated = propagate_polygon(node->polygon_lon,
                                                                polygon_zero_state_lon,
                                                                config->planning().dt,
                                                                config->vehicle().ego.v_lon_min,
                                                                config->vehicle().ego.v_lon_max);

                auto polygon_lat_propagated = propagate_polygon(node->polygon_lat,
                                                                polygon_zero_state_lat,
                                                                config->planning().dt,
                                                                config->vehicle().ego.v_lat_min,
                                                                config->vehicle().ego.v_lat_max);

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

/// *Steps*:
/// 1. create a list of new base sets cut down with rectangles of the drivable area.
/// 2. create the list of nodes of the new reachable set.
void ReachableSet::_compute_reachable_set_at_time_step(int const& time_step) {
    auto drivable_area = map_time_to_drivable_area[time_step];
    auto vec_base_sets_propagated = map_time_to_base_set_propagated[time_step];
    if (drivable_area.empty())
        return;

    auto vec_base_sets_adapted = adapt_base_sets_to_drivable_area(drivable_area, vec_base_sets_propagated,
                                                                  config->reachable_set().num_threads);

    auto reachable_set_current_step = create_reachable_set_nodes(time_step, vec_base_sets_adapted,
                                                                 config->reachable_set().num_threads);

    map_time_to_reachable_set[time_step] = reachable_set_current_step;
}
