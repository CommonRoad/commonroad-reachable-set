from typing import List

import numpy as np
import pytest
from shapely.geometry.point import Point

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_analysis import ReachabilityAnalysis
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import reach_operation


def test_generate_tuples_vertices_initial(config: Configuration):
    list_vertices_p_lon, list_vertices_p_lat = reach_operation.generate_tuples_vertices_polygons_initial(config)
    # values from the scenario DEU_Test-1_1_T-1
    if config.planning.coordinate_system == "CART":
        p_lon = 35.1
        p_lat = 2.1
        v_lon = 12.0
        v_lat = 0

    elif config.planning.coordinate_system == "CVLN":
        p_lon = 35.1
        p_lat = 0.1
        v_lon = 12.0
        v_lat = 0

    else:
        raise Exception("Coordinate not defined.")

    dp = dv = 0.01
    atol = 1e-04

    tuple_coords_expected_p_lon = (p_lon - dp, v_lon - dv, p_lon + dp, v_lon + dv)
    for vertex in tuple_coords_expected_p_lon:
        is_close = False
        for vertex_drivable_area in list_vertices_p_lon:
            is_close = is_close or np.allclose(vertex, vertex_drivable_area, atol=atol)

        assert is_close

    tuple_coords_expected_lat = (p_lat - dp, v_lat - dv, p_lat + dp, v_lat + dv)
    for vertex in tuple_coords_expected_lat:
        is_close = False
        for vertex_drivable_area in list_vertices_p_lat:
            is_close = is_close or np.allclose(vertex, vertex_drivable_area, atol=atol)

        assert is_close


def test_create_bounding_polygon_returns_correct_vertices():
    polygon = reach_operation.create_bounding_polygon(dt=2, a_min=-5, a_max=10)

    list_vertices_polygon = polygon.vertices
    list_vertices_expected = [(-10, -10), (20, -10), (-10, 20), (20, 20)]
    for vertex in list_vertices_expected:
        assert vertex in list_vertices_polygon


def test_create_zero_state_polygon_returns_correct_vertices():
    polygon = reach_operation.create_zero_state_polygon(dt=2, a_min=-2.0, a_max=2.0)

    list_vertices_polygon = polygon.vertices
    list_vertices_expected = [(4, 4), (-4, -4), (0, 2), (0, -2), (-4, -2), (4, 2)]
    for vertex in list_vertices_expected:
        assert vertex in list_vertices_polygon


def test_propagate_polygon(config: Configuration):
    config.planning.dt = 2.0

    config.vehicle.ego.v_lon_min = 0.0
    config.vehicle.ego.v_lon_max = 20.0
    config.vehicle.ego.a_lon_min = -2.0
    config.vehicle.ego.a_lon_max = 2.0

    config.vehicle.ego.v_lat_min = 0.0
    config.vehicle.ego.v_lat_max = 20.0
    config.vehicle.ego.a_lat_min = -2.0
    config.vehicle.ego.a_lat_max = 2.0

    list_vertices = [[10, 0], [30, 0], [30, 20], [10, 20], [10, 0]]
    list_vertices_expected = [(72.0, 20.0),
                              (70.0, 18.0),
                              (34.0, 0.0),
                              (8.0, 0.0),
                              (10.0, 2.0),
                              (46.0, 20.0),
                              (72.0, 20.0)]

    reachability_analysis = ReachabilityAnalysis(config)
    polygon_lon = ReachPolygon(list_vertices)
    polygon_propagated = reach_operation.propagate_polygon(polygon_lon,
                                                           reachability_analysis.polygon_zero_state_lon,
                                                           config.planning.dt,
                                                           config.vehicle.ego.v_lon_min,
                                                           config.vehicle.ego.v_lon_max)
    assert set(polygon_propagated.vertices) == set(list_vertices_expected)

    polygon_lat = ReachPolygon(list_vertices)
    polygon_propagated = reach_operation.propagate_polygon(polygon_lat,
                                                           reachability_analysis.polygon_zero_state_lat,
                                                           config.planning.dt,
                                                           config.vehicle.ego.v_lat_min,
                                                           config.vehicle.ego.v_lat_max)
    assert set(polygon_propagated.vertices) == set(list_vertices_expected)


def test_compute_minimum_positions_of_polygons(list_polygons_lon: List[ReachPolygon],
                                               list_polygons_lat: List[ReachPolygon]):
    p_lon_min_expected = 2.0
    p_lat_min_expected = -2.1

    list_base_sets_propagated = [ReachNode(list_polygons_lon[0], list_polygons_lat[0])]
    list_polygons_projected = [base_set.position_rectangle for base_set in list_base_sets_propagated]
    p_lon_min, p_lat_min = reach_operation.compute_minimum_positions_of_rectangles(list_polygons_projected)

    assert np.isclose(p_lon_min, p_lon_min_expected) and np.isclose(p_lat_min, p_lat_min_expected)


@pytest.mark.parametrize("size_grid, tuple_coords_expected", [(0.5, (0, 0, 22, 17)), (0.2, (0, 0, 54, 42))])
def test_create_discretized_position_rectangles(list_polygons_lon: List[ReachPolygon],
                                                list_polygons_lat: List[ReachPolygon],
                                                size_grid,
                                                tuple_coords_expected):
    list_base_sets_propagated = [ReachNode(list_polygons_lon[0], list_polygons_lat[0])]
    list_polygons_projected = [base_set.position_rectangle for base_set in list_base_sets_propagated]
    p_lon_min, p_lat_min = reach_operation.compute_minimum_positions_of_rectangles(list_polygons_projected)

    list_rectangles_discretized = reach_operation.discretize_rectangles(list_polygons_projected,
                                                                        (p_lon_min, p_lat_min),
                                                                        size_grid)
    rectangle = list_rectangles_discretized[0]

    assert tuple_coords_expected == rectangle.bounds


@pytest.mark.parametrize("size_grid, tuple_coords_expected", [(0.5, (3.5, 4.5, 11, 13.5)), (0.2, (3.2, 3.6, 6.2, 7.2))])
def test_create_undiscretized_position_rectangles(list_rectangles_discritized, size_grid, tuple_coords_expected):
    p_lon_min = 3
    p_lat_min = 3

    list_rectangles_undiscritized = reach_operation.undiscretized_rectangles(list_rectangles_discritized,
                                                                             (p_lon_min, p_lat_min),
                                                                             size_grid)
    rectangle = list_rectangles_undiscritized[0]

    assert tuple_coords_expected == rectangle.bounds


def test_create_base_set_from_position_region():
    rectangle_drivable_area = ReachPolygon.from_rectangle_vertices(0, 0, 10, 10)
    list_polygons_lon = [ReachPolygon.from_rectangle_vertices(-5, 10, 5, 15),
                         ReachPolygon.from_rectangle_vertices(5, 0, 15, 20)]

    list_polygons_lat = [ReachPolygon.from_rectangle_vertices(-3, -5, 3, 5),
                         ReachPolygon.from_rectangle_vertices(3, 0, 13, 12)]
    list_base_sets = [ReachNode(polygon_lon, polygon_lat)
                      for polygon_lon, polygon_lat in zip(list_polygons_lon, list_polygons_lat)]
    list_idx_base_sets_adjacent = [0, 1]

    list_vertices_lon_expected = [(5, 10), (9.5, 0.5), (9.5, 14.5), (9.5, 19.5)]
    list_vertices_lat_expected = [(2, -4), (5, 2), (5, 7), (5, 11.9)]

    base_set_adapted = reach_operation.adapt_base_set_to_drivable_area(rectangle_drivable_area,
                                                                       list_idx_base_sets_adjacent, list_base_sets)

    for vertex in list_vertices_lon_expected:
        assert base_set_adapted.polygon_lon.contains(Point(vertex))

    for vertex in list_vertices_lat_expected:
        assert base_set_adapted.polygon_lat.contains(Point(vertex))
