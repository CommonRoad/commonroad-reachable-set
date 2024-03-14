from commonroad_reach.data_structure.segment_tree import CounterSegmentTree, CounterTreeNode, ToggleSegmentTree


def test_activate_joint_segments():
    tree = CounterSegmentTree(0, 20)
    tree.activate(0, 10)
    tree.activate(5, 15)

    list_nodes_active = tree.get_active_nodes()
    for node_active in [CounterTreeNode(0, 10), CounterTreeNode(10, 15)]:
        assert node_active in list_nodes_active


def test_activate_disjoint_segments():
    tree = CounterSegmentTree(0, 20)
    tree.activate(0, 5)
    tree.activate(15, 20)

    list_nodes_active = tree.get_active_nodes()
    for node_active in [CounterTreeNode(0, 5), CounterTreeNode(15, 20)]:
        assert node_active in list_nodes_active

    tree.activate(5, 15)
    list_nodes_active = tree.get_active_nodes()
    for node_active in [CounterTreeNode(0, 20)]:
        assert node_active in list_nodes_active


def test_deactivate_after_activate_returns_empty_active_nodes():
    tree = CounterSegmentTree(0, 20)

    tree.activate(0, 10)
    tree.activate(5, 15)
    tree.deactivate(5, 15)
    tree.deactivate(0, 10)

    assert len(tree.get_active_nodes()) == 0

    tree.activate(0, 10)
    tree.activate(5, 15)
    tree.deactivate(0, 10)
    tree.deactivate(5, 15)

    assert len(tree.get_active_nodes()) == 0


def test_toggle_joint_segments():
    tree = ToggleSegmentTree(0, 20)

    tree.toggle(10, 15)
    tree.toggle(12, 18)
    stack_interval_active = tree.get_stack_of_active_intervals()
    assert stack_interval_active == [10, 12, 15, 18]


def test_toggle_disjoint_segments():
    tree = ToggleSegmentTree(0, 20)

    tree.toggle(5, 10)
    tree.toggle(15, 19)
    stack_interval_active = tree.get_stack_of_active_intervals()
    assert stack_interval_active == [5, 10, 15, 19]
