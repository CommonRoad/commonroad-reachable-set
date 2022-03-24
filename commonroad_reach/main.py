import commonroad_reach.utility.geometry as util_geometry
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


def main():
    list_vertices_polygon_1 = [(-1, -1), (1, -1), (0, 1)]
    list_vertices_polygon_2 = [(3, -1), (5, -1), (5, 1), (3, 1)]
    list_vertices_polygon_summed_expected = [(2.0, -2.0), (4.0, -2.0), (6.0, -2.0), (6.0, -0.0), (5.0, 2.0), (3.0, 2.0),
                                             (2.0, -0.0)]

    polygon_1 = ReachPolygon(list_vertices_polygon_1)
    polygon_2 = ReachPolygon(list_vertices_polygon_2)
    polygon_1 = ReachPolygon.from_polygon(polygon_1.convex_hull)
    polygon_2 = ReachPolygon.from_polygon(polygon_2.convex_hull)

    polygon_summed_1 = util_geometry.minkowski_sum2(polygon_1, polygon_2)
    polygon_summed_2 = util_geometry.minkowski_sum2(polygon_2, polygon_1)

    list_vertices_summed_1 = [tuple(vertex) for vertex in polygon_summed_1.vertices]
    list_vertices_summed_2 = [tuple(vertex) for vertex in polygon_summed_2.vertices]

    assert set(list_vertices_summed_1) == set(list_vertices_summed_2)
    for vertex in list_vertices_polygon_summed_expected:
        assert vertex in list_vertices_summed_1


if __name__ == "__main__":
    main()
