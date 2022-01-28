from collections import defaultdict
from typing import Dict, List, Tuple, Union
import math

import commonroad_dc.pycrcc as pycrcc
import numpy as np
import skgeom as sg
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from skgeom import minkowski

try:
    import pycrreachset
except ImportError:
    pass


def linear_mapping(polygon, tuple_coefficients: Tuple):
    """Returns the linear mapping of polygon.

    Computing the zero-input response of the system, see Söntges T-ITS 2018, Sec. IV.A.
    """
    list_vertices = polygon.vertices
    a11, a12, a21, a22 = tuple_coefficients

    list_vertices_mapped = [(a11 * vertex[0] + a12 * vertex[1], a21 * vertex[0] + a22 * vertex[1])
                            for vertex in list_vertices]

    return ReachPolygon(list_vertices_mapped)


def minkowski_sum(polygon1: ReachPolygon, polygon2: ReachPolygon) -> ReachPolygon:
    """Returns the Minkowski sum of two polygons.

    Args:
        polygon1 (ReachPolygon): first polygon
        polygon2 (ReachPolygon): second polygon
    """
    # shapely polygon requires identical initial and final vertices
    # scikit-geometry polygon requires different initial and final vertices
    # minkowski sum in scikit-geometry requires sorting counterclockwise
    list_vertices = sort_vertices_counterclockwise([vertex for vertex in polygon1.vertices])
    sg_polygon_1 = sg.Polygon(list_vertices)

    list_vertices = sort_vertices_counterclockwise([vertex for vertex in polygon2.vertices])
    sg_polygon_2 = sg.Polygon(list_vertices)

    result = minkowski.minkowski_sum(sg_polygon_1, sg_polygon_2)
    list_vertices_sum = [list(vertex) for vertex in result.outer_boundary().coords]

    return ReachPolygon(list_vertices_sum)


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


def create_adjacency_dictionary(
        list_rectangles_1: List[ReachPolygon], list_rectangles_2: List[ReachPolygon]) -> Dict[int, List[int]]:
    """Returns an adjacency dictionary.

    E.g.: {0:[1, 2], [1:[3, 4]]} = rectangle_0 from 1st list overlaps
    with rectangles_1 and _2 from the 2nd list; rectangle_1 (1st) overlaps
    with rectangles_3 and _4 (2nd).
    """
    dict_idx_to_list_idx = defaultdict(list)

    for idx_1, rectangle_1 in enumerate(list_rectangles_1):
        for idx_2, rectangle_2 in enumerate(list_rectangles_2):
            if not (rectangle_1.p_lon_min > rectangle_2.p_lon_max or rectangle_1.p_lon_max < rectangle_2.p_lon_min
                    or rectangle_1.p_lat_min > rectangle_2.p_lat_max or rectangle_1.p_lat_max < rectangle_2.p_lat_min):
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


def area_of_reachable_set(list_reach_set_nodes: List[Union[pycrreachset.ReachNode, ReachNode]]) -> float:
    """
    Function computes the area of a given list of reachable set nodes
    :param list_reach_set_nodes: List of base sets (i.e., reachable set nodes)
    :return area of reachable set
    """
    area = 0.0
    if type(list_reach_set_nodes[0]) == pycrreachset.ReachNode:
        for node in list_reach_set_nodes:
            area += (node.p_lon_max() - node.p_lon_min()) * (node.p_lat_max() - node.p_lat_min())
    else:
        for node in list_reach_set_nodes:
            area += (node.p_lon_max - node.p_lon_min) * (node.p_lat_max - node.p_lat_min)
    return area


def connected_reachset_py(list_reach_set_nodes: List[ReachNode], no_of_digits: int):
    """
    Function determines connected sets in the position domain within a given list of reachable set nodes
    This function is the equivalent python function to pycrreachset.connected_reachset_boost().
    Returns a dictionary with key=node idx in list and value=list of tuples
    :param list_reach_set_nodes: list of reachable set node
    :param no_of_digits
    """
    coeff = math.pow(10.0, no_of_digits)
    overlap = defaultdict(list)

    # list of drivable areas (i.e., position rectangles)
    list_position_rectangles = list()

    # preprocess
    for reach_node in list_reach_set_nodes:
        # enlarge position rectangles
        vertices_rectangle_scaled = (reach_node.p_lon_min * coeff,
                                     reach_node.p_lat_min * coeff,
                                     reach_node.p_lon_max * coeff,
                                     reach_node.p_lat_max * coeff)
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
