#include "reachset/data_structure/reach/reach_node.hpp"

#include <utility>
#include "reachset/utility/shared_using.hpp"

using namespace reach;

int ReachNode::cnt_id = 0;

ReachNode::ReachNode(int const& step, ReachPolygonPtr polygon_lon, ReachPolygonPtr polygon_lat) :
        step(step), polygon_lon(std::move(polygon_lon)), polygon_lat(std::move(polygon_lat)) {
    id = ReachNode::cnt_id++;
}

bool ReachNode::add_parent_node(ReachNodePtr const& node_parent) {
    if (std::none_of(_vec_nodes_parent.cbegin(), _vec_nodes_parent.cend(),
                     [&](auto const& node) { return node.lock() == node_parent; })) {
        std::weak_ptr<ReachNode> node_parent_weak_ptr = node_parent;
        _vec_nodes_parent.emplace_back(node_parent_weak_ptr);
        return true;
    }

    return false;
}


bool ReachNode::remove_parent_node(ReachNodePtr const& node_parent) {
    auto it_end = std::remove_if(_vec_nodes_parent.begin(), _vec_nodes_parent.end(), [&node_parent](const std::weak_ptr<ReachNode>& weakPtr) {
            return weakPtr.lock() == node_parent; // Compare using lock()
    });
    if (it_end != _vec_nodes_parent.end()) {
        _vec_nodes_parent.erase(it_end, _vec_nodes_parent.end());
        return true;
    }

    return false;
}

bool ReachNode::add_child_node(ReachNodePtr const& node_child) {
    if (std::none_of(_vec_nodes_child.cbegin(),
                     _vec_nodes_child.cend(),
                     [&](auto const& node) { return node.lock() == node_child; })) {
        std::weak_ptr<ReachNode> node_child_weak_ptr = node_child;
        _vec_nodes_child.emplace_back(node_child_weak_ptr);
        return true;
    }

    return false;
}

bool ReachNode::remove_child_node(ReachNodePtr const& node_child) {
    auto it_end = std::remove_if(_vec_nodes_child.begin(), _vec_nodes_child.end(), [&node_child](const std::weak_ptr<ReachNode>& weakPtr) {
            return weakPtr.lock() == node_child; // Compare using lock()
    });
    if (it_end != _vec_nodes_child.end()) {
        _vec_nodes_child.erase(it_end, _vec_nodes_child.end());
        return true;
    }

    return false;
}