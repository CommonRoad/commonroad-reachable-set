from commonroad_reach.data_structure.reach.reach_node import ReachNode
from shapely.geometry import Polygon


def test_new_node_has_id_0():
    ReachNode.reset_class_id_counter()
    node = ReachNode(None, None, 0)
    assert node.id == 0


def test_new_node_has_empty_parents_and_children(node: ReachNode):
    assert len(node.list_nodes_parent) == 0 and len(node.list_nodes_child) == 0


def test_node_has_correct_boundaries():
    polygon_lon = Polygon([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]])
    polygon_lat = Polygon([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]])
    idx_time = 0
    node = ReachNode(polygon_lon, polygon_lat, idx_time)
    assert (
            node.p_lon_min == 0
            and node.p_lon_max == 1
            and node.v_lon_min == 0
            and node.v_lon_max == 1
            and node.p_lat_min == 0
            and node.p_lat_max == 1
            and node.v_lat_min == 0
            and node.v_lat_max == 1
    )


def test_removing_valid_parent_returns_true(node: ReachNode):
    node_parent = ReachNode(None, None, 0)
    node.add_parent_node(node_parent)
    result = node.remove_parent_node(node_parent)
    assert result


def test_removing_same_valid_parent_twice_returns_false(node: ReachNode):
    node_parent = ReachNode(None, None, 0)
    node.add_parent_node(node_parent)
    node.remove_parent_node(node_parent)
    result = node.remove_parent_node(node_parent)
    assert not result


def test_removing_invalid_parent_returns_false(node: ReachNode):
    node_parent = ReachNode(None, None, 0)
    result = node.remove_parent_node(node_parent)
    assert not result


def test_removing_valid_child_returns_true(node: ReachNode):
    node_child = ReachNode(None, None, 0)
    node.add_child_node(node_child)
    result = node.remove_child_node(node_child)
    assert result


def test_removing_same_valid_child_twice_returns_false(node: ReachNode):
    node_child = ReachNode(None, None, 0)
    node.add_child_node(node_child)
    node.remove_child_node(node_child)
    result = node.remove_child_node(node_child)
    assert not result


def test_removing_invalid_child_returns_false(node: ReachNode):
    node_child = ReachNode(None, None, 0)
    result = node.remove_child_node(node_child)
    assert not result
