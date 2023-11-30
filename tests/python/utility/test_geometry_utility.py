from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import geometry as util_geometry


def test_linear_mapping():
    list_vertices = [[10, 0], [30, 0], [30, 20], [10, 20], [10, 0]]
    polygon = ReachPolygon(list_vertices)
    tuple_coefficients = (1, 0.1, 0, 1)
    polygon_mapped = util_geometry.linear_mapping(polygon, tuple_coefficients)

    list_vertices_polygon = polygon_mapped.vertices
    list_vertices_expected = [(10, 0), (30, 0), (32, 20), (12, 20)]
    for vertex in list_vertices_expected:
        assert vertex in list_vertices_polygon


def test_minkowski_sum():
    list_vertices_polygon_1 = [(-1, -1), (1, -1), (0, 1)]
    list_vertices_polygon_2 = [(3, -1), (5, -1), (5, 1), (3, 1)]
    list_vertices_polygon_summed_expected = [(2.0, -2.0), (6.0, -2.0), (6.0, -0.0), (5.0, 2.0), (3.0, 2.0),
                                             (2.0, -0.0)]

    polygon_1 = ReachPolygon(list_vertices_polygon_1)
    polygon_2 = ReachPolygon(list_vertices_polygon_2)
    polygon_1 = ReachPolygon.from_polygon(polygon_1.convex_hull)
    polygon_2 = ReachPolygon.from_polygon(polygon_2.convex_hull)

    polygon_summed_1 = util_geometry.minkowski_sum(polygon_1, polygon_2)
    polygon_summed_2 = util_geometry.minkowski_sum(polygon_2, polygon_1)

    list_vertices_summed_1 = [tuple(vertex) for vertex in polygon_summed_1.vertices]
    list_vertices_summed_2 = [tuple(vertex) for vertex in polygon_summed_2.vertices]

    assert set(list_vertices_summed_1) == set(list_vertices_summed_2)
    for vertex in list_vertices_polygon_summed_expected:
        assert vertex in list_vertices_summed_1


def test_create_adjacency_dictionary():
    list_tuples_vertices_1 = [(1, 0, 2, 1), (2, 0, 3, 1)]
    list_tuples_vertices_2 = [(0.5, 0.5, 1.5, 1.5), (1.5, 0.5, 2.5, 1.5), (2.5, 0.5, 3.5, 1.5)]
    dict_adjacency_expected = {0: [0, 1], 1: [1, 2]}

    list_rectangles_1 = [ReachPolygon.from_rectangle_vertices(*tuple_vertices)
                         for tuple_vertices in list_tuples_vertices_1]

    list_rectangles_2 = [ReachPolygon.from_rectangle_vertices(*tuple_vertices)
                         for tuple_vertices in list_tuples_vertices_2]

    dict_adjacency = util_geometry.create_adjacency_dictionary(list_rectangles_1, list_rectangles_2)

    assert dict_adjacency == dict_adjacency_expected


def test_examine_overlapping_relationship():
    list_rectangles_1 = [ReachPolygon.from_rectangle_vertices(0.5, 0.5, 1.5, 1.5),
                         ReachPolygon.from_rectangle_vertices(1.5, 0.5, 2.5, 1.5)]

    list_rectangles_2 = [ReachPolygon.from_rectangle_vertices(0, 0, 1, 1),
                         ReachPolygon.from_rectangle_vertices(1, 0, 2, 1),
                         ReachPolygon.from_rectangle_vertices(2, 0, 3, 1)]

    dict_id_rectangle_1_to_list_ids_rectangles_2 = \
        util_geometry.create_adjacency_dictionary(list_rectangles_1, list_rectangles_2)

    assert dict_id_rectangle_1_to_list_ids_rectangles_2.get(0) == [0, 1] and \
           dict_id_rectangle_1_to_list_ids_rectangles_2.get(1) == [1, 2]
