from collections import defaultdict
from typing import Dict, List, Tuple, Optional

import numpy as np
import commonroad_dc.pycrcc as pycrcc

from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.data_structure.reach.reach_vertex import Vertex


def linear_mapping(polygon: ReachPolygon, tuple_coefficients: Tuple[float, float, float, float]) -> ReachPolygon:
    """
    Returns the linear mapping of the input polygon.
    """
    list_vertices = polygon.vertices
    a11, a12, a21, a22 = tuple_coefficients

    list_vertices_mapped = [(a11 * vertex[0] + a12 * vertex[1],
                             a21 * vertex[0] + a22 * vertex[1]) for vertex in list_vertices]

    return ReachPolygon(list_vertices_mapped)


def minkowski_sum(polygon1: ReachPolygon, polygon2: ReachPolygon) -> Optional[ReachPolygon]:
    """
    Returns the Minkowski sum of the two input polygons.
    """
    if polygon1.is_empty or polygon2.is_empty:
        return None

    list_vertices_sum = list()
    list_vertices_1 = sort_vertices_counterclockwise(polygon1.vertices)
    list_vertices_2 = sort_vertices_counterclockwise(polygon2.vertices)

    for vertex_1 in list_vertices_1:
        for vertex_2 in list_vertices_2:
            list_vertices_sum.append(Vertex(vertex_1[0] + vertex_2[0], vertex_1[1] + vertex_2[1]))

    list_vertices_sum = list(set(list_vertices_sum))
    list_vertices_sum = [(vertex.x, vertex.y) for vertex in list_vertices_sum]

    return ReachPolygon.from_polygon(ReachPolygon(list_vertices_sum).shapely_object.convex_hull)


def sort_vertices_counterclockwise(list_vertices: List[Tuple]) -> List[Tuple[float, float]]:
    """
    Sorts a list of vertices in the counterclockwise direction.

    Steps:
        1. compute the center of mass
        2. compute the angle from the center of mass to each point
        3. sort with the computed angles
    """
    list_x = np.array([vertex[0] for vertex in list_vertices])
    list_y = np.array([vertex[1] for vertex in list_vertices])

    x_mean = np.mean(list_x)
    y_mean = np.mean(list_y)

    r = np.sqrt((list_x - x_mean) ** 2 + (list_y - y_mean) ** 2)

    list_angles = np.where((list_y - y_mean) > 0, np.arccos((list_x - x_mean) / r),
                           2 * np.pi - np.arccos((list_x - x_mean) / r))

    mask = np.argsort(list_angles)

    list_x_sorted = list_x[mask]
    list_y_sorted = list_y[mask]
    list_vertices_sorted = [(x, y) for x, y in zip(list_x_sorted, list_y_sorted)]

    return list_vertices_sorted


def create_adjacency_dictionary(list_rectangles_1: List[ReachPolygon],
                                list_rectangles_2: List[ReachPolygon]) -> Dict[int, List[int]]:
    """
    Returns an adjacency dictionary from the two given list of rectangles.

    .. note::
        **Example**: {0 : [1, 2], 1 : [3, 4]} = rectangle_0 from 1st list overlaps with rectangles_1 and _2 from the
        2nd list; rectangle_1 from the 1st list overlaps with rectangles_3 and _4 from the 2nd list.
    """
    dict_idx_to_list_idx = defaultdict(list)

    for idx_1, rectangle_1 in enumerate(list_rectangles_1):
        for idx_2, rectangle_2 in enumerate(list_rectangles_2):
            if not (rectangle_1.p_lon_min >= rectangle_2.p_lon_max or
                    rectangle_1.p_lon_max <= rectangle_2.p_lon_min or
                    rectangle_1.p_lat_min >= rectangle_2.p_lat_max or
                    rectangle_1.p_lat_max <= rectangle_2.p_lat_min):
                dict_idx_to_list_idx[idx_1].append(idx_2)

    return dict_idx_to_list_idx


def obtain_extremum_coordinates_of_vertices(list_vertices: List[Tuple]) -> Tuple[float, float, float, float]:
    """
    Returns the extremum coordinates of the given list of vertices.
    """
    list_p_lon = [vertex[0] for vertex in list_vertices]
    list_p_lat = [vertex[1] for vertex in list_vertices]

    p_lon_min = min(list_p_lon)
    p_lon_max = max(list_p_lon)
    p_lat_min = min(list_p_lat)
    p_lat_max = max(list_p_lat)

    return p_lon_min, p_lat_min, p_lon_max, p_lat_max


def create_aabb_from_coordinates(p_lon_min: float, p_lat_min: float, p_lon_max: float, p_lat_max: float):
    """
    Returns a pycrcc.RectAABB object from the given coordinates.
    """
    length = p_lon_max - p_lon_min
    width = p_lat_max - p_lat_min
    center_lon = (p_lon_min + p_lon_max) / 2.0
    center_lat = (p_lat_min + p_lat_max) / 2.0

    aabb = pycrcc.RectAABB(length / 2.0, width / 2.0, center_lon, center_lat)

    return aabb


def rectangle_intersects_with_circle(rectangle: ReachPolygon, center: Tuple[float, float], radius: float) -> bool:
    """
    Returns true if the given rectangles intersects with a circle.

    :param rectangle: rectangle to be checked
    :param center: center of the circle to be checked
    :param radius: radius of the circle to be checked"""
    x_closest = clamp(center[0], rectangle.p_lon_min, rectangle.p_lon_max)
    y_closest = clamp(center[1], rectangle.p_lat_min, rectangle.p_lat_max)

    distance_x = center[0] - x_closest
    distance_y = center[1] - y_closest

    distance_squared = (distance_x * distance_x) + (distance_y * distance_y)

    return distance_squared < (radius * radius)


def clamp(value: float, min_value: float, max_value: float):
    """
    Clamps a value between the min and max values.
    """
    if value <= min_value:
        return min_value

    elif value >= max_value:
        return max_value

    else:
        return value

