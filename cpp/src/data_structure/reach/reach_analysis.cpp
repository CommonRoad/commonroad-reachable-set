#include "reachset/data_structure/reach/reach_analysis.hpp"

#include <utility>
#include "reachset/utility/shared_using.hpp"
#include "reachset/utility/reach_operation.hpp"

using namespace reach;

ContinuousReachabilityAnalysis::ContinuousReachabilityAnalysis(ConfigurationPtr config) : _config(std::move(config)) {
    initialize_zero_state_polygons();
}

ContinuousReachabilityAnalysis::ContinuousReachabilityAnalysis(
        ConfigurationPtr config, CollisionCheckerPtr collision_checker) :
        _config(std::move(config)), _collision_checker(std::move(collision_checker)) {
    initialize_zero_state_polygons();
}

/// @note Computation of the reachable set of an LTI system requires the zero-state response and
/// zero-input response of the system.
void ContinuousReachabilityAnalysis::initialize_zero_state_polygons() {
    this->_polygon_zero_state_lon = create_zero_state_polygon(_config->planning().dt,
                                                              _config->vehicle().ego.a_lon_min,
                                                              _config->vehicle().ego.a_lon_max);

    this->_polygon_zero_state_lat = create_zero_state_polygon(_config->planning().dt,
                                                              _config->vehicle().ego.a_lat_min,
                                                              _config->vehicle().ego.a_lat_max);
}

vector<ReachPolygonPtr> reach::ContinuousReachabilityAnalysis::initial_drivable_area() const {
    vector<ReachPolygonPtr> vec_polygon;

    auto tuple_vertices = generate_tuple_vertices_position_rectangle_initial(_config);
    vec_polygon.emplace_back(std::apply(ReachPolygon::from_rectangle_coordinates, tuple_vertices));

    return vec_polygon;

}

std::vector<ReachNodePtr> ContinuousReachabilityAnalysis::initial_reachable_set() const {
    vector<ReachNodePtr> vec_node;

    auto[tuple_vertices_polygon_lon, tuple_vertices_polygon_lat] = generate_tuples_vertices_polygons_initial(_config);
    auto polygon_lon = std::apply(ReachPolygon::from_rectangle_coordinates, tuple_vertices_polygon_lon);
    auto polygon_lat = std::apply(ReachPolygon::from_rectangle_coordinates, tuple_vertices_polygon_lat);
    vec_node.emplace_back(make_shared<ReachNode>(_config->planning().time_step_start, polygon_lon, polygon_lat));

    return vec_node;
}

/// *Steps*:
/// 1. Propagate each node of the reachable set from the last time step. This forms a list of propagated base sets.
/// 2. Project the base sets onto the position domain.
/// 3. Merge and repartition these rectangles to reduce computation load.
/// 4. Check for collision and split the repartitioned rectangles into collision-free rectangles.
/// 5. Merge and repartition the collision-free rectangles again to reduce number of nodes.
std::tuple<std::vector<ReachPolygonPtr>, std::vector<ReachNodePtr>>
ContinuousReachabilityAnalysis::compute_drivable_area_at_time_step(
        int const& time_step, vector<ReachNodePtr> const& reachable_set_time_step_previous) {

    if (reachable_set_time_step_previous.empty())
        return {{},
                {}};

    // Step 1
    auto vec_base_sets_propagated = propagate_reachable_set(reachable_set_time_step_previous);

    // Step 2
    auto vec_rectangles_projected = project_base_sets_to_position_domain(vec_base_sets_propagated);

    // Step 3
    auto vec_rectangles_repartitioned = create_repartitioned_rectangles(vec_rectangles_projected,
                                                                        _config->reachable_set().size_grid);
    // Step 4
    auto vec_rectangles_collision_free = check_collision_and_split_rectangles(
            std::floor(time_step * _config->planning().dt * 10),
            _collision_checker,
            vec_rectangles_repartitioned,
            _config->reachable_set().radius_terminal_split,
            _config->reachable_set().num_threads);

    // Step 5
    auto drivable_area = create_repartitioned_rectangles(vec_rectangles_collision_free,
                                                         _config->reachable_set().size_grid_2nd);

    return {drivable_area, vec_base_sets_propagated};
}

vector<ReachNodePtr>
ContinuousReachabilityAnalysis::propagate_reachable_set(vector<ReachNodePtr> const& vec_nodes) {
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

/// *Steps*:
/// 1. create a list of new base sets cut down with rectangles of the drivable area.
/// 2. create the list of nodes of the new reachable set.
vector<ReachNodePtr> ContinuousReachabilityAnalysis::compute_reachable_set_at_time_step(
        int const& time_step, vector<ReachNodePtr> const& vec_base_sets_propagated,
        vector<ReachPolygonPtr> const& drivable_area) {
    if (drivable_area.empty())
        return {};

    // Step 1
    auto vec_base_sets_adapted = adapt_base_sets_to_drivable_area(drivable_area, vec_base_sets_propagated,
                                                                  _config->reachable_set().num_threads);

    // Step 2
    auto reachable_set_time_step_current = create_reachable_set_nodes_continuous(time_step, vec_base_sets_adapted,
                                                                                 _config->reachable_set().num_threads);
    return reachable_set_time_step_current;
}