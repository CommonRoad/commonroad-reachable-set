import pytest

from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


def test_creating_polygon_with_less_than_three_vertices_throws_exception():
    list_vertices = [(10, 0), (30, 0)]
    with pytest.raises(Exception):
        ReachPolygon(list_vertices)


def test_has_correct_bounding_box_upon_initialization():
    list_vertices = [(0, 0), (5, -5), (10, 10)]
    polygon = ReachPolygon(list_vertices)
    assert polygon._bounds == (0, -5, 10, 10)


def test_convexification():
    list_vertices = [(0, 0), (5, -5), (10, 10), (5, 2), (5, -2)]
    polygon = ReachPolygon(list_vertices)
    polygon = ReachPolygon.from_polygon(polygon.shapely_object.convex_hull)

    assert len(polygon.vertices) == 3
    list_vertices_expected = [(0, 0), (5, -5), (10, 10)]
    for vertex in list_vertices_expected:
        assert vertex in polygon.vertices


def test_intersect_halfspace_general():
    """Intersection with general halfspace ax + by <= c (with a, b != 0)"""
    list_vertices = [(10, 0), (30, 0), (30, 20), (10, 20), (10, 0)]
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


def test_intersect_halfspace_vertical_positive_a():
    """Intersection with vertical halfspace ax <= c, b = 0 and a > 0"""
    list_vertices = [(-10, -10), (-10, 10), (10, 10), (10, -10), (-10, -10)]
    polygon = ReachPolygon(list_vertices)

    print(f"\n\nFirst cut with halfspace 2x <= 10")  # x <= 5
    a, b, c = (2, 0, 10)
    polygon_intersection = polygon.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection.vertices
    list_vertices_expected = [(5, -10), (-10, -10), (-10, 10), (5, 10)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected

    print(f"\nSecond cut with halfspace 2x <= 0")  # x <= 0
    a, b, c = (2, 0, 0)
    polygon_intersection_2 = polygon_intersection.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection_2.vertices
    list_vertices_expected = [(0, -10), (-10, -10), (-10, 10), (0, 10)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected

    print(f"\nThird cut with halfspace 2x <= -10")  # x <= -5
    a, b, c = (2, 0, -10)
    polygon_intersection_3 = polygon_intersection_2.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection_3.vertices
    list_vertices_expected = [(-5, -10), (-10, -10), (-10, 10), (-5, 10)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected


def test_intersect_halfspace_vertical_negative_a():
    """Intersection with vertical halfspace ax <= c, b = 0 and a < 0"""
    list_vertices = [(-10, -10), (-10, 10), (10, 10), (10, -10), (-10, -10)]
    polygon = ReachPolygon(list_vertices)

    print(f"\n\nFirst cut with halfspace -2x <= 10")  # x >= -5
    a, b, c = (-2, 0, 10)
    polygon_intersection = polygon.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection.vertices
    list_vertices_expected = [(10, -10), (-5, -10), (-5, 10), (10, 10)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected

    print(f"\nSecond cut with halfspace -2x <= 0")  # x >= 0
    a, b, c = (-2, 0, 0)
    polygon_intersection_2 = polygon_intersection.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection_2.vertices
    list_vertices_expected = [(10, -10), (0, -10), (0, 10), (10, 10)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected

    print(f"\nThird cut with halfspace -2x <= -10")  # x >= 5
    a, b, c = (-2, 0, -10)
    polygon_intersection_3 = polygon_intersection_2.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection_3.vertices
    list_vertices_expected = [(10, -10), (5, -10), (5, 10), (10, 10)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected


def test_intersect_halfspace_horizontal_positive_b():
    """Intersection with horizontal halfspace by <= c, a = 0 and b > 0"""
    list_vertices = [(-10, -10), (-10, 10), (10, 10), (10, -10), (-10, -10)]
    polygon = ReachPolygon(list_vertices)

    print(f"\n\nFirst cut with halfspace 2y <= 10")  # y <= 5
    a, b, c = (0, 2, 10)
    polygon_intersection = polygon.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection.vertices
    list_vertices_expected = [(-10, -10), (-10, 5), (10, 5), (10, -10)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected

    print(f"\nSecond cut with halfspace 2y <= 0")  # y <= 0
    a, b, c = (0, 2, 0)
    polygon_intersection_2 = polygon_intersection.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection_2.vertices
    list_vertices_expected = [(-10, -10), (-10, 0), (10, 0), (10, -10)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected

    print(f"\nThird cut with halfspace 2y <= -10")  # y <= -5
    a, b, c = (0, 2, -10)
    polygon_intersection_3 = polygon_intersection_2.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection_3.vertices
    list_vertices_expected = [(-10, -10), (-10, -5), (10, -5), (10, -10)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected


def test_intersect_halfspace_horizontal_negative_b():
    """Intersection with horizontal halfspace by <= c, a = 0 and b < 0"""
    list_vertices = [(-10, -10), (-10, 10), (10, 10), (10, -10), (-10, -10)]
    polygon = ReachPolygon(list_vertices)

    print(f"\n\nFirst cut with halfspace 2y <= 10")  # y >= -5
    a, b, c = (0, -2, 10)
    polygon_intersection = polygon.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection.vertices
    list_vertices_expected = [(-10, -5), (-10, 10), (10, 10), (10, -5)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected

    print(f"\nSecond cut with halfspace 2y <= 0")  # y >= 0
    a, b, c = (0, -2, 0)
    polygon_intersection_2 = polygon_intersection.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection_2.vertices
    list_vertices_expected = [(-10, 0), (-10, 10), (10, 10), (10, 0)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected

    print(f"\nThird cut with halfspace 2y <= -10")  # y >= 5
    a, b, c = (0, -2, -10)
    polygon_intersection_3 = polygon_intersection_2.intersect_halfspace(a, b, c)
    list_vertices_polygon = polygon_intersection_3.vertices
    list_vertices_expected = [(-10, 5), (-10, 10), (10, 10), (10, 5)]
    for vertex in list_vertices_polygon:
        assert vertex in list_vertices_expected
