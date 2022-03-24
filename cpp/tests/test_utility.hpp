#pragma once

#include <doctest/doctest.h>
#include "reachset/utility/shared_include.hpp"

#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reachset/data_structure/reach/reach_node.hpp"
#include "reachset/data_structure/segment_tree.hpp"

#include "reachset/utility/reach_operation.hpp"
#include "reachset/utility/sweep_line.hpp"
#include "reachset/utility/shared_using.hpp"
#include "reachset/utility/collision_checker.hpp"

#include "reachset/data_structure/reach/reach_set.hpp"

using namespace reach;
using namespace collision;
using namespace geometry;

bool vertex_in_vertices(std::tuple<double, double> const& tuple_vertex, std::vector<Vertex> const& vertices);

bool counter_node_in_nodes(CounterTreeNode const& node, std::vector<CounterTreeNode> const& vec_nodes);

bool event_in_events(SweepLine::Event const& event, std::vector<SweepLine::Event> const& vec_events);

bool segment_in_segments(ReachLine const& segment, std::vector<ReachLine> const& vec_segments);

bool bound_in_bounds(tuple<double, double, double, double> const& bound,
                     std::vector<tuple<double, double, double, double>> vec_bounds);

bool vertex_within_polygon(std::tuple<double, double> const& tuple_vertex, ReachPolygonPtr const& polygon);