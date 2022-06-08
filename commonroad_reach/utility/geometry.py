import math
from collections import defaultdict
from typing import Dict, List, Tuple, Optional

import commonroad_dc.pycrcc as pycrcc
import numpy as np

from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.data_structure.reach.reach_vertex import Vertex


def linear_mapping(polygon, tuple_coefficients: Tuple):
    """Returns the linear mapping of polygon.

    Computing the zero-input response of the system, see Söntges T-ITS 2018, Sec. IV.A.
    """
    list_vertices = polygon.vertices
    a11, a12, a21, a22 = tuple_coefficients

    list_vertices_mapped = [(a11 * vertex[0] + a12 * vertex[1], a21 * vertex[0] + a22 * vertex[1])
                            for vertex in list_vertices]

    return ReachPolygon(list_vertices_mapped)


def minkowski_sum(polygon1: ReachPolygon, polygon2: ReachPolygon) -> Optional[ReachPolygon]:
    """Returns the Minkowski sum of two polygons."""
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

    return ReachPolygon.from_polygon(ReachPolygon(list_vertices_sum).convex_hull)


def sort_vertices_counterclockwise(list_vertices: List[Tuple]) -> List[Tuple]:
    """Sorts a list of vertices in the counterclockwise direction.

    Steps:
        1. compute the center of mass
        2. compute the angles from the center of mass to each point
        3. sort with the computed angles

    Args:
        list_vertices (List[Tuple]): the list of vertices to be sorted
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


def create_adjacency_dictionary(list_rectangles_1: List[ReachPolygon], list_rectangles_2: List[ReachPolygon]) \
        -> Dict[int, List[int]]:
    """Returns an adjacency dictionary.

    E.g.: {0 : [1, 2], 1 : [3, 4]} = rectangle_0 from 1st list overlaps with rectangles_1 and _2 from the 2nd list;
    rectangle_1 (1st) overlaps with rectangles_3 and _4 (2nd).
    """
    dict_idx_to_list_idx = defaultdict(list)

    for idx_1, rectangle_1 in enumerate(list_rectangles_1):
        for idx_2, rectangle_2 in enumerate(list_rectangles_2):
            if rectangle_1.intersects(rectangle_2):
                dict_idx_to_list_idx[idx_1].append(idx_2)

    return dict_idx_to_list_idx


def obtain_extremum_coordinates_of_vertices(list_vertices):
    """Returns the extremum coordinates of the given vertices."""
    list_p_lon = [vertex[0] for vertex in list_vertices]
    list_p_lat = [vertex[1] for vertex in list_vertices]

    p_lon_min = min(list_p_lon)
    p_lon_max = max(list_p_lon)
    p_lat_min = min(list_p_lat)
    p_lat_max = max(list_p_lat)

    return p_lon_min, p_lat_min, p_lon_max, p_lat_max


def create_aabb_from_coordinates(p_lon_min, p_lat_min, p_lon_max, p_lat_max):
    """Returns a pycrcc.RectAABB object from the given coordinates."""
    length = p_lon_max - p_lon_min
    width = p_lat_max - p_lat_min
    center_lon = (p_lon_min + p_lon_max) / 2.0
    center_lat = (p_lat_min + p_lat_max) / 2.0

    aabb = pycrcc.RectAABB(length / 2.0, width / 2.0, center_lon, center_lat)

    return aabb


def rectangle_intersects_with_circle(rectangle: ReachPolygon, center: Tuple[float, float], radius: float) -> bool:
    """Returns true if the given rectangles intersects with the attributes of a circle."""
    x_closest = clamp(center[0], rectangle.p_lon_min, rectangle.p_lon_max)
    y_closest = clamp(center[1], rectangle.p_lat_min, rectangle.p_lat_max)

    distance_x = center[0] - x_closest
    distance_y = center[1] - y_closest

    distance_squared = (distance_x * distance_x) + (distance_y * distance_y)

    return distance_squared < (radius * radius)


def clamp(value, min_value, max_value):
    if value <= min_value:
        return min_value

    elif value >= max_value:
        return max_value

    else:
        return value


def area_of_reachable_set(list_reach_set_nodes: List[ReachNode]) -> float:
    """
    Computes the area of a given list of reachable set nodes.
    :param list_reach_set_nodes:
    :return area: area of projection of the reachable sets on position domain
    """
    area = 0.0
    if isinstance(list_reach_set_nodes[0], ReachNode):
        for node in list_reach_set_nodes:
            area += (node.p_lon_max - node.p_lon_min) * (node.p_lat_max - node.p_lat_min)

    else:
        for node in list_reach_set_nodes:
            area += (node.p_lon_max() - node.p_lon_min()) * (node.p_lat_max() - node.p_lat_min())

    return area


def connected_reachset_py(list_reach_set_nodes: List[ReachNode], no_of_digits: int):
    """
    Function determines connected sets in the position domain within a given list of reachable set nodes
    This function is the equivalent python function to pycrreach.connected_reachset_boost().
    Returns a dictionary with key=node idx in list and value=list of tuples
    :param list_reach_set_nodes: list of reachable set node
    :param no_of_digits
    """
    coefficient = math.pow(10.0, no_of_digits)
    overlap = defaultdict(list)

    # list of drivable areas (i.e., position rectangles)
    list_position_rectangles = list()

    # preprocess
    for reach_node in list_reach_set_nodes:
        # enlarge position rectangles
        vertices_rectangle_scaled = (math.floor(reach_node.p_lon_min * coefficient),
                                     math.floor(reach_node.p_lat_min * coefficient),
                                     math.ceil(reach_node.p_lon_max * coefficient),
                                     math.ceil(reach_node.p_lat_max * coefficient))
        list_position_rectangles.append(ReachPolygon.from_rectangle_vertices(*vertices_rectangle_scaled))

    # iterate over all rectangles in list
    for idx1, position_rect_1 in enumerate(list_position_rectangles):
        # remove index of position_rect_1 to avoid checking self-intersection
        check_idx_list = list(range(len(list_position_rectangles)))
        check_idx_list.pop(idx1)
        # iterate over all other rectangles
        for idx2 in check_idx_list:
            # retrieve second rectangle
            position_rect_2 = list_position_rectangles[idx2]
            # check for overlap via shapely intersects() function. If True, add tuple of idx to dict
            if position_rect_1.intersects(position_rect_2):
                overlap[idx1].append((idx1, idx2))

    return overlap
