#include "test_utility.hpp"
#include "reachset/data_structure/segment_tree.hpp"

using namespace reach;

TEST_SUITE("TestSegmentTree") {
TEST_CASE("CounterSegmentTree") {
    auto tree = CounterSegmentTree(0, 20);

    SUBCASE("activate joint segments") {
        tree.activate(0, 10);
        tree.activate(5, 15);

        auto vec_nodes_active = tree.get_active_nodes();
        vector<CounterTreeNode> vec_nodes_unwrapped;
        vec_nodes_unwrapped.reserve(vec_nodes_active.size());
        for (auto const& node:vec_nodes_active)
            vec_nodes_unwrapped.emplace_back(*node);

        for (auto const& node_active:{CounterTreeNode(0, 10), CounterTreeNode(10, 15)}) {
            CHECK(counter_node_in_nodes(node_active, vec_nodes_unwrapped));
        }
    }

    SUBCASE("activate disjoint segments") {
        tree.activate(0, 5);
        tree.activate(15, 20);

        auto vec_nodes_active = tree.get_active_nodes();
        vector<CounterTreeNode> vec_nodes_unwrapped;
        vec_nodes_unwrapped.reserve(vec_nodes_active.size());
        for (auto const& node:vec_nodes_active)
            vec_nodes_unwrapped.emplace_back(*node);

        for (auto const& node_active:{CounterTreeNode(0, 5), CounterTreeNode(15, 20)}) {
            CHECK(counter_node_in_nodes(node_active, vec_nodes_unwrapped));
        }

        tree.activate(5, 15);

        vec_nodes_active = tree.get_active_nodes();
        vec_nodes_unwrapped.clear();
        vec_nodes_unwrapped.reserve(vec_nodes_active.size());
        for (auto const& node:vec_nodes_active)
            vec_nodes_unwrapped.emplace_back(*node);

        for (auto const& node_active:{CounterTreeNode(0, 20)}) {
            CHECK(counter_node_in_nodes(node_active, vec_nodes_unwrapped));
        }

    }

    SUBCASE("deactivate after activate returns empty active nodes") {
        tree.activate(0, 10);
        tree.activate(5, 15);
        tree.deactivate(5, 15);
        tree.deactivate(0, 10);

        CHECK(tree.get_active_nodes().empty());

        tree.activate(0, 10);
        tree.activate(5, 15);
        tree.deactivate(0, 10);
        tree.deactivate(5, 15);

        CHECK(tree.get_active_nodes().empty());;

    }
}
}