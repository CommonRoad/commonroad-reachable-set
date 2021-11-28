from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.data_structure.reach.reach_node import ReachNode


def test_project_to_position_domain():
    polygon_lon = ReachPolygon.from_rectangle_vertices(0, 0, 5, 10)
    polygon_lat = ReachPolygon.from_rectangle_vertices(7, -5, 15, 5)
    node = ReachNode(polygon_lon, polygon_lat)

    assert node.position_rectangle.bounds == (0, 7, 5, 15)
