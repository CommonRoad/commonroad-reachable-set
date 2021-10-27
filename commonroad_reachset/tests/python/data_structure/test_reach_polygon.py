import pytest
from commonroad_reachset.data_structure.reach.reach_polygon import ReachPolygon


def test_creating_polygon_with_less_than_three_vertices_throws_exception():
    list_vertices = [[10, 0], [30, 0]]
    with pytest.raises(Exception):
        ReachPolygon(list_vertices)


def test_intersect_halfspace():
    list_vertices = [[10, 0], [30, 0], [30, 20], [10, 20], [10, 0]]
    polygon = ReachPolygon(list_vertices)

    # first cut
    polygon_intersected = polygon.intersect_halfspace(1, -1, 20)

    list_vertices_polygon = polygon_intersected.vertices
    list_vertices_expected = [(20, 0), (30, 10)]
    for vertex in list_vertices_expected:
        assert vertex in list_vertices_polygon
    # second cut
    polygon_intersected = polygon_intersected.intersect_halfspace(0.5, 1, 25)

    list_vertices_polygon = polygon_intersected.vertices
    list_vertices_expected = [(10, 20)]
    for vertex in list_vertices_expected:
        assert vertex in list_vertices_polygon
    # third cut
    polygon_intersected = polygon_intersected.intersect_halfspace(-2, -1, -30)

    list_vertices_polygon = polygon_intersected.vertices
    list_vertices_expected = [(10, 10), (15, 0)]
    for vertex in list_vertices_expected:
        assert vertex in list_vertices_polygon

    # final assertion
    list_vertices_expected = [(15, 0), (20, 0), (30, 10), (10, 20), (10, 10)]
    for vertex in list_vertices_expected:
        assert vertex in list_vertices_polygon
