#include "test_utility.hpp"

TEST_SUITE("TestReachNode") {
TEST_CASE("node initialization") {
    ReachNode::reset_id_counter();
    auto node = ReachNode(0, nullptr, nullptr);

    SUBCASE("new node has id 0") {
        CHECK(node.id == 0);
    }

    SUBCASE("new node has empty parents and children") {
        CHECK(node.vec_nodes_parent().size() == 0);
        CHECK(node.vec_nodes_child().size() == 0);
    }
}

TEST_CASE("add/remove parent/child nodes") {
    auto polygon_lon = ReachPolygon::from_rectangle_coordinates(0, 0, 5, 10);
    auto polygon_lat = ReachPolygon::from_rectangle_coordinates(7, -5, 15, 5);
    auto node = ReachNode{0, polygon_lon, polygon_lat};

    auto node_parent = std::make_shared<ReachNode>(0, nullptr, nullptr);
    auto node_child = std::make_shared<ReachNode>(1, nullptr, nullptr);

    SUBCASE("removing valid parent node returns true") {
        node.add_parent_node(node_parent);
        auto result = node.remove_parent_node(node_parent);
        CHECK(result);
    }

    SUBCASE("removing same valid parent node twice returns false") {
        node.add_parent_node(node_parent);
        node.remove_parent_node(node_parent);
        auto result = node.remove_parent_node(node_parent);
        CHECK(not result);
    }

    SUBCASE("removing invalid parent node returns false") {
        auto result = node.remove_parent_node(node_parent);
        CHECK(not result);
    }
    SUBCASE("removing valid child node returns true") {
        node.add_child_node(node_child);
        auto result = node.remove_child_node(node_child);
        CHECK(result);
    }

    SUBCASE("removing same valid child node twice returns false") {
        node.add_child_node(node_child);
        node.remove_child_node(node_child);
        auto result = node.remove_child_node(node_child);
        CHECK(not result);
    }

    SUBCASE("removing invalid child node returns false") {
        auto result = node.remove_child_node(node_child);
        CHECK(not result);
    }
}
}