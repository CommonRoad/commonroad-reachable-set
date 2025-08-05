#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "test_utility.hpp"

bool vertex_in_vertices(std::tuple<double, double> const& tuple_vertex, std::vector<Vertex> const& vertices) {
    auto result = std::any_of(vertices.begin(), vertices.end(),
                              [&](Vertex const& vertex) {
                                  return vertex.x == std::get<0>(tuple_vertex) and
                                         vertex.y == std::get<1>(tuple_vertex);
                              });

    if (not result)
        std::cout << "Failed at vertex: (" << std::get<0>(tuple_vertex) << ", " << std::get<1>(tuple_vertex) << ")" << std::endl;

    return result;
}

bool counter_node_in_nodes(CounterTreeNode const& node, std::vector<CounterTreeNode> const& vec_nodes) {
    auto result = std::find(vec_nodes.begin(), vec_nodes.end(), node) != vec_nodes.end();

    if (not result)
        std::cout << "Failed at node: (" << node.low << ", " << node.high << ")" << std::endl;

    return result;
}

bool event_in_events(SweepLine::Event const& event, std::vector<SweepLine::Event> const& vec_events) {
    auto result = std::find(vec_events.begin(), vec_events.end(), event) != vec_events.end();

    if (not result)
        std::cout << "Failed at event: (" << event.p_lat_low << ", " << event.p_lat_high << ")" << std::endl;

    return result;
}

bool segment_in_segments(ReachLine const& segment, std::vector<ReachLine> const& vec_segments) {
    auto result = std::find(vec_segments.begin(), vec_segments.end(), segment) != vec_segments.end();

    if (not result)
        std::cout << "Failed at segment: (" << segment.p_lon_min << ", " << segment.p_lat_min << ", "
             << segment.p_lon_max << ", " << segment.p_lat_max << ")" << std::endl;

    return result;
}

bool bound_in_bounds(std::tuple<double, double, double, double> const& bound,
                     std::vector<std::tuple<double, double, double, double>> vec_bounds) {
    auto result = std::find(vec_bounds.begin(), vec_bounds.end(), bound) != vec_bounds.end();

    if (not result)
        std::cout << "Failed at bound: (" << std::get<0>(bound) << ", " << std::get<1>(bound) << ", "
             << std::get<2>(bound) << ", " << std::get<3>(bound) << ")" << std::endl;

    return result;
}