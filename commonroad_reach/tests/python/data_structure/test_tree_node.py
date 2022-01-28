from commonroad_reach.data_structure.segment_tree import CounterTreeNode


def test_new_tree_node_has_empty_children():
    node = CounterTreeNode(20, 50)
    assert not node.child_left and not node.child_right


def test_tree_node_enclosed_by_interval():
    node = CounterTreeNode(20, 50)

    assert node.enclosed_by_interval(20, 50)
    assert node.enclosed_by_interval(10, 60)
    assert node.enclosed_by_interval(10, 50)
    assert node.enclosed_by_interval(20, 50)
    assert not node.enclosed_by_interval(21, 50)
    assert not node.enclosed_by_interval(20, 49)
