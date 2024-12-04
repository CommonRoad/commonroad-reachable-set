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
    step_start = config->planning().step_start;
    step_end = step_start + config->planning().steps_computation;
    map_step_to_drivable_area[step_start] = _construct_initial_drivable_area();
    map_step_to_reachable_set[step_start] = _construct_initial_reachable_set();

    _vec_steps_computed.emplace_back(step_start);
    _initialize_zero_state_polygons();
}

vector<ReachPolygonPtr> ReachableSet::_construct_initial_drivable_area() const {
    vector<ReachPolygonPtr> vec_polygon;

    auto tuple_vertices = generate_tuple_vertices_position_rectangle_initial(config);
    vec_polygon.emplace_back(make_shared<ReachPolygon>(std::get<0>(tuple_vertices),
                                                       std::get<1>(tuple_vertices),
                                                       std::get<2>(tuple_vertices),
                                                       std::get<3>(tuple_vertices)));
    return vec_polygon;
}

std::vector<ReachNodePtr> ReachableSet::_construct_initial_reachable_set() const {
    vector<ReachNodePtr> vec_node;

    auto [tuple_vertices_polygon_lon, tuple_vertices_polygon_lat] =
            generate_tuples_vertices_polygons_initial(config);
    auto polygon_lon = make_shared<ReachPolygon>(tuple_vertices_polygon_lon);
    auto polygon_lat = make_shared<ReachPolygon>(tuple_vertices_polygon_lat);
    vec_node.emplace_back(make_shared<ReachNode>(config->planning().step_start, polygon_lon, polygon_lat));

    return vec_node;
}

/// @note Computation of the reachable set of an LTI system requires the zero-state response of the system.
void ReachableSet::_initialize_zero_state_polygons() {
    polygon_zero_state_lon = create_zero_state_polygon(config->planning().dt,
                                                       config->vehicle().ego.a_lon_min,
                                                       config->vehicle().ego.a_lon_max);

    polygon_zero_state_lat = create_zero_state_polygon(config->planning().dt,
                                                       config->vehicle().ego.a_lat_min,
                                                       config->vehicle().ego.a_lat_max);
}


void ReachableSet::compute(int step_start, int step_end) {
    if (step_start == 0) step_start = this->step_start + 1;
    if (step_end == 0) step_end = this->step_end;

    for (auto step = step_start; step < step_end + 1; step++) {
        _compute_drivable_area_at_step(step);
        _compute_reachable_set_at_step(step);
        _vec_steps_computed.emplace_back(step);
    }

    if (step_start != step_end and config->reachable_set().prune_nodes) {
        prune_nodes_not_reaching_final_step();
    }
}

/// *Steps*:
/// 1. Propagate each node of the reachable set from the last step. This forms a list of propagated base sets.
/// 2. Project the base sets onto the position domain.
/// 3. Merge and repartition these rectangles to reduce computation load.
/// 4. Check for collision and split the repartitioned rectangles into collision-free rectangles.
/// 5. Merge and repartition the collision-free rectangles again to reduce number of nodes.
void ReachableSet::_compute_drivable_area_at_step(int const& step) {
    auto reachable_set_previous = map_step_to_reachable_set[step - 1];
    if (reachable_set_previous.empty()) {
        map_step_to_drivable_area[step] = vector<ReachPolygonPtr>{};
        map_step_to_base_set_propagated[step] = vector<ReachNodePtr>{};
        return;
    }

    auto vec_base_sets_propagated = _propagate_reachable_set(reachable_set_previous);

    auto vec_rectangles_projected = project_base_sets_to_position_domain(vec_base_sets_propagated);

    vector<ReachPolygonPtr> drivable_area_collision_free{};
    // repartition, then collision check
    if (config->reachable_set().mode_repartition == 1) {
        auto vec_rectangles_repartitioned = create_repartitioned_rectangles(
                vec_rectangles_projected, config->reachable_set().size_grid);
        if (config->reachable_set().mode_inflation != 3) {
            drivable_area_collision_free = check_collision_and_split_rectangles(
                    step, collision_checker,
                    vec_rectangles_repartitioned, config->reachable_set().radius_terminal_split,
                    config->reachable_set().num_threads);
        } else {
            drivable_area_collision_free = check_collision_and_split_rectangles(
                    step, collision_checker, vec_rectangles_repartitioned,
                    config->reachable_set().radius_terminal_split,
                    config->reachable_set().num_threads,
                    config->vehicle().ego.circle_distance,
                    *config->planning().CLCS,
                    *config->reachable_set().lut_lon_enlargement,
                    config->planning().reference_point);
        }
        // collision check, then repartition
    } else if (config->reachable_set().mode_repartition == 2) {
        vector<ReachPolygonPtr> vec_rectangles_collision_free{};
        if (config->reachable_set().mode_inflation != 3) {
            vec_rectangles_collision_free = check_collision_and_split_rectangles(
                    step, collision_checker,
                    vec_rectangles_projected, config->reachable_set().radius_terminal_split,
                    config->reachable_set().num_threads);
        } else {
            vec_rectangles_collision_free = check_collision_and_split_rectangles(
                    step, collision_checker, vec_rectangles_projected,
                    config->reachable_set().radius_terminal_split,
                    config->reachable_set().num_threads,
                    config->vehicle().ego.circle_distance,
                    *config->planning().CLCS,
                    *config->reachable_set().lut_lon_enlargement,
                    config->planning().reference_point);
        }
        drivable_area_collision_free = create_repartitioned_rectangles(
                vec_rectangles_collision_free, config->reachable_set().size_grid);
        // repartition, collision check, then repartition again
    } else if (config->reachable_set().mode_repartition == 3) {
        auto vec_rectangles_repartitioned = create_repartitioned_rectangles(
                vec_rectangles_projected, config->reachable_set().size_grid);

        vector<ReachPolygonPtr> vec_rectangles_collision_free{};
        if (config->reachable_set().mode_inflation != 3) {
            vec_rectangles_collision_free = check_collision_and_split_rectangles(
                    step, collision_checker,
                    vec_rectangles_repartitioned, config->reachable_set().radius_terminal_split,
                    config->reachable_set().num_threads);
        } else {
            vec_rectangles_collision_free = check_collision_and_split_rectangles(
                    step, collision_checker, vec_rectangles_projected,
                    config->reachable_set().radius_terminal_split,
                    config->reachable_set().num_threads,
                    config->vehicle().ego.circle_distance,
                    *config->planning().CLCS,
                    *config->reachable_set().lut_lon_enlargement,
                    config->planning().reference_point);
        }

        drivable_area_collision_free = create_repartitioned_rectangles(
                vec_rectangles_collision_free, config->reachable_set().size_grid);

    } else throw std::logic_error("Invalid mode for repartition.");

    map_step_to_drivable_area[step] = drivable_area_collision_free;
    map_step_to_base_set_propagated[step] = vec_base_sets_propagated;
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

                auto base_set_propagated = make_shared<ReachNode>(node->step,
                                                                  polygon_lon_propagated,
                                                                  polygon_lat_propagated);
                std::weak_ptr<ReachNode> node_weak_ptr = node;
                base_set_propagated->vec_nodes_source.emplace_back(node_weak_ptr);
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
/// 1. construct reach nodes from drivable area and the propagated base sets.
/// 2. update parent-child relationship of the nodes.
void ReachableSet::_compute_reachable_set_at_step(int const& step) {
    auto drivable_area = map_step_to_drivable_area[step];
    auto vec_base_sets_propagated = map_step_to_base_set_propagated[step];
    auto num_threads = config->reachable_set().num_threads;

    if (drivable_area.empty()) {
        map_step_to_reachable_set[step] = vector<ReachNodePtr>{};
        return;
    }

    auto vec_nodes = construct_reach_nodes(drivable_area, vec_base_sets_propagated, num_threads);

    auto reachable_set = connect_children_to_parents(step, vec_nodes, num_threads);

    map_step_to_reachable_set[step] = reachable_set;
}

/// Iterates through reachability graph backward in time, discards nodes that don't have a child node.
void ReachableSet::prune_nodes_not_reaching_final_step() {
    auto cnt_nodes_before_pruning = reachable_set_at_step(step_end).size();
    auto cnt_nodes_after_pruning = cnt_nodes_before_pruning;

    for (auto step = step_end - 1; step > step_start - 1; step--) {
        auto vec_nodes = reachable_set_at_step(step);
        cnt_nodes_before_pruning += vec_nodes.size();

        vector<int> vec_idx_nodes_to_be_deleted{};
        for (int idx_node = 0; idx_node < vec_nodes.size(); idx_node++) {
            // discard the node if it has no child node
            auto node = vec_nodes[idx_node];
            if (node->vec_nodes_child().empty()) {
                vec_idx_nodes_to_be_deleted.push_back(idx_node);
                // iterate through its parent nodes and disconnect them
                for (auto const& node_parent: node->vec_nodes_parent()) {
                    node_parent->remove_child_node(node);
                }
            }
        }
        // discard nodes without a child
        vector<ReachPolygonPtr> vec_drivable_area_updated{};
        vector<ReachNodePtr> vec_reachable_set_updated{};
        for (int idx_node = 0; idx_node < vec_nodes.size(); idx_node++) {
            auto result = std::find(vec_idx_nodes_to_be_deleted.begin(),
                                    vec_idx_nodes_to_be_deleted.end(),
                                    idx_node) != vec_idx_nodes_to_be_deleted.end();

            if (not result) {
                auto node = vec_nodes[idx_node];
                vec_drivable_area_updated.emplace_back(node->position_rectangle());
                vec_reachable_set_updated.emplace_back(node);
            }
        }
        // update drivable area and reachable set dictionaries
        map_step_to_drivable_area[step] = vec_drivable_area_updated;
        map_step_to_reachable_set[step] = vec_reachable_set_updated;
        cnt_nodes_after_pruning += map_step_to_reachable_set[step].size();
    }

    _pruned = true;
    // cout << "\t#Nodes before pruning: \t" << cnt_nodes_before_pruning << endl;
    // cout << "\t#Nodes after pruning: \t" << cnt_nodes_after_pruning << endl;
}
