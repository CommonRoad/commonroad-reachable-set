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
    auto node = ReachNode(0, nullptr, nullptr);
    auto node_parent = make_shared<ReachNode>(0, nullptr, nullptr);
    auto node_child = make_shared<ReachNode>(1, nullptr, nullptr);

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

TEST_CASE("position rectangle") {
    ReachNode::reset_id_counter();
    auto polygon_lon = make_shared<ReachPolygon>(0, 0, 5, 10);
    auto polygon_lat = make_shared<ReachPolygon>(7, -5, 15, 5);
    auto node = ReachNode(0, polygon_lon, polygon_lat);

    SUBCASE("has correct position rectangle") {
        CHECK(node.position_rectangle()->bounding_box() == tuple(0, 7, 5, 15));
    }

    SUBCASE("has correct vertices") {
        CHECK(node.p_lon_min() == 0);
        CHECK(node.p_lon_max() == 5);
        CHECK(node.v_lon_min() == 0);
        CHECK(node.v_lon_max() == 10);
        CHECK(node.p_lat_min() == 7);
        CHECK(node.p_lat_max() == 15);
        CHECK(node.v_lat_min() == -5);
        CHECK(node.v_lat_max() == 5);
    }

    SUBCASE("intersection in position domain") {
        node.intersect_in_position_domain(1, 8, 4, 20);
        CHECK(node.position_rectangle()->bounding_box() == tuple(1, 8, 4, 15));
    }

    // reset node
    node = ReachNode(0, polygon_lon, polygon_lat);
    SUBCASE("intersection in position domain with infinite bounds") {
        double inf = std::numeric_limits<double>::infinity();
        node.intersect_in_position_domain(1, -inf, inf, inf);
        REQUIRE_EQ(node.position_rectangle()->bounding_box(), tuple(1, 7, 5, 15));

        node.intersect_in_position_domain(-inf, 8, inf, inf);
        REQUIRE_EQ(node.position_rectangle()->bounding_box(), tuple(1, 8, 5, 15));

        node.intersect_in_position_domain(-inf, -inf, 4, inf);
        REQUIRE_EQ(node.position_rectangle()->bounding_box(), tuple(1, 8, 4, 15));

        node.intersect_in_position_domain(-inf, -inf, inf, 20);
        REQUIRE_EQ(node.position_rectangle()->bounding_box(), tuple(1, 8, 4, 15));
    }
}
}