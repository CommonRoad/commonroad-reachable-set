#include "reachset/common/data_structure/reach_node.hpp"
#include "reachset/common/utility/shared_using.hpp"

using namespace reach;

int ReachNode::cnt_id = 0;

ReachNode::ReachNode(int const& time_step, ReachPolygonPtr const& polygon_lon, ReachPolygonPtr const& polygon_lat,
                     set<string> const& set_propositions) :
        time_step(time_step), polygon_lon(polygon_lon), polygon_lat(polygon_lat), set_propositions(set_propositions) {
    id = ReachNode::cnt_id++;
    if (polygon_lon != nullptr) box_lon = polygon_lon->bounding_box();
    if (polygon_lat != nullptr) box_lat = polygon_lat->bounding_box();
}

ReachNodePtr ReachNode::clone() const {
    auto node_clone = make_shared<ReachNode>(time_step, polygon_lon->clone(),
                                             polygon_lat->clone(), set_propositions);
    node_clone->vec_nodes_child() = vec_nodes_child();
    node_clone->vec_nodes_parent() = vec_nodes_parent();
    node_clone->vec_nodes_source = vec_nodes_source;

    return node_clone;
}

ReachPolygonPtr ReachNode::position_rectangle() const {
    return ReachPolygon::from_rectangle_coordinates(p_lon_min(), p_lat_min(),
                                                    p_lon_max(), p_lat_max());
}

bool ReachNode::add_parent_node(ReachNodePtr const& node_parent) {
    if (std::none_of(_vec_nodes_parent.cbegin(),
                     _vec_nodes_parent.cend(),
                     [&](auto const& node) { return node == node_parent; })) {
        _vec_nodes_parent.emplace_back(node_parent);
        return true;
    }

    return false;
}


bool ReachNode::remove_parent_node(ReachNodePtr const& node_parent) {
    auto it_end = std::remove(_vec_nodes_parent.begin(), _vec_nodes_parent.end(), node_parent);
    if (it_end != _vec_nodes_parent.end()) {
        _vec_nodes_parent.erase(it_end, _vec_nodes_parent.end());
        return true;
    }

    return false;
}

bool ReachNode::add_child_node(ReachNodePtr const& node_child) {
    if (std::none_of(_vec_nodes_child.cbegin(),
                     _vec_nodes_child.cend(),
                     [&](auto const& node) { return node == node_child; })) {
        _vec_nodes_child.emplace_back(node_child);
        return true;
    }

    return false;
}

bool ReachNode::remove_child_node(ReachNodePtr const& node_child) {
    auto it_end = std::remove(_vec_nodes_child.begin(), _vec_nodes_child.end(), node_child);
    if (it_end != _vec_nodes_child.end()) {
        _vec_nodes_child.erase(it_end, _vec_nodes_child.end());
        return true;
    }

    return false;
}

void ReachNode::assign_parent_nodes(std::vector<std::shared_ptr<ReachNode>> const& vec_nodes_parent) {
    _vec_nodes_parent = vec_nodes_parent;
}

void ReachNode::assign_child_nodes(std::vector<std::shared_ptr<ReachNode>> const& vec_nodes_child) {
    _vec_nodes_child = vec_nodes_child;
}

void ReachNode::clear_parent_nodes() {
    this->_vec_nodes_parent.clear();
}

void ReachNode::clear_child_nodes() {
    this->_vec_nodes_child.clear();
}

void ReachNode::reset_id_counter() {
    ReachNode::cnt_id = 0;
}

void ReachNode::intersect_in_position_domain(double const& p_lon_min, double const& p_lat_min,
                                             double const& p_lon_max, double const& p_lat_max) {
    polygon_lon->intersect_halfspace(1, 0, p_lon_max);
    polygon_lon->intersect_halfspace(-1, 0, -p_lon_min);
    polygon_lat->intersect_halfspace(1, 0, p_lat_max);
    polygon_lat->intersect_halfspace(-1, 0, -p_lat_min);
}

void ReachNode::intersect_in_velocity_domain(double const& v_lon_min, double const& v_lat_min,
                                             double const& v_lon_max, double const& v_lat_max) {
    polygon_lon->intersect_halfspace(0, 1, v_lon_max);
    polygon_lon->intersect_halfspace(0, -1, -v_lon_min);
    polygon_lat->intersect_halfspace(0, 1, v_lat_max);
    polygon_lat->intersect_halfspace(0, -1, -v_lat_min);
}
