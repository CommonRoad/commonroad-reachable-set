import logging
from collections import defaultdict
from decimal import Decimal
from typing import Union

import numpy as np
import networkx as nx

logger = logging.getLogger("REACH_LOGGER")
from math import ceil, floor
from typing import List, Tuple

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.regular_grid import Grid, Cell
from commonroad_reach.data_structure.reach.reach_node import ReachNode, ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import geometry as util_geometry
from commonroad_reach.utility.sweep_line import SweepLine
from commonroad_reach import pycrreach


def create_zero_state_polygon(dt: float, a_min: float, a_max: float) -> ReachPolygon:
    """
    Returns the zero-state polygon of the system.

    Steps:
        1. prepare a bounding polygon to be intersected with halfspaces.
        2. compute coefficients of halfspaces and intersect them with the bounding polygon.
           We use three halfspaces to approximate the upper bound of the polygon, this applies to the lower bound as well
           A halfspace can be specified given a switching time (gamma).

    :param dt: time duration (one step)
    :param a_min: minimum acceleration (maximum deceleration)
    :param a_max: maximum acceleration
    """
    polygon_bounding = create_bounding_polygon(dt, a_min, a_max)

    polygon_intersected = polygon_bounding
    for gamma in [0, 0.5, 1]:
        tuple_coefficients_upper, tuple_coefficients_lower = compute_halfspace_coefficients(dt, a_min, a_max, gamma)

        a, b, c = tuple_coefficients_upper
        polygon_intersected = polygon_intersected.intersect_halfspace(a, b, c)

        a, b, c = tuple_coefficients_lower
        polygon_intersected = polygon_intersected.intersect_halfspace(a, b, c)

    return polygon_intersected


def create_bounding_polygon(dt: float, a_min: float, a_max: float) -> ReachPolygon:
    """
    Returns a polygon that has the min/max reachable position and velocity as its bounds.

    :param dt: time duration (one step)
    :param a_min: minimum acceleration (maximum deceleration)
    :param a_max: maximum acceleration
    """
    tuple_vertices = (0.5 * a_min * (dt ** 2), a_min * dt, 0.5 * a_max * (dt ** 2), a_max * dt)

    return ReachPolygon.from_rectangle_vertices(*tuple_vertices)


def compute_halfspace_coefficients(dt: float, a_min: float, a_max: float, gamma: float) -> Tuple:
    """
    Computes the coefficients of halfspaces to be intersected.

    :param dt: time duration (one step)
    :param a_min: minimum acceleration (maximum deceleration)
    :param a_max: maximum acceleration
    :param gamma: switching time between full braking and acceleration
    """
    x_upper = a_max * dt ** 2 * (0.5 - gamma + 0.5 * gamma ** 2) + a_min * dt ** 2 * (gamma - 0.5 * gamma ** 2)
    v_upper = a_min * gamma * dt + a_max * dt * (1 - gamma)
    dx_upper = a_max * dt ** 2 * (-1 + gamma) + a_min * dt ** 2 * (1 - gamma)
    dv_upper = a_min * dt - a_max * dt

    x_lower = a_min * dt ** 2 * (0.5 - gamma + 0.5 * gamma ** 2) + a_max * dt ** 2 * (gamma - 0.5 * gamma ** 2)
    v_lower = a_max * gamma * dt + a_min * dt * (1 - gamma)
    dx_lower = a_min * dt ** 2 * (-1 + gamma) + a_max * dt ** 2 * (1 - gamma)
    dv_lower = a_max * dt - a_min * dt

    tuple_coefficients_upper = (dv_upper, -dx_upper, dv_upper * x_upper - dx_upper * v_upper)
    tuple_coefficients_lower = (dv_lower, -dx_lower, dv_lower * x_lower - dx_lower * v_lower)

    return tuple_coefficients_upper, tuple_coefficients_lower


def generate_tuple_vertices_position_rectangle_initial(config: Configuration) -> Tuple:
    """
    Returns a tuple of vertices for the position rectangle construction.
    """
    config = config.planning

    tuple_vertices = (config.p_lon_initial - config.uncertainty_p_lon, config.p_lat_initial - config.uncertainty_p_lat,
                      config.p_lon_initial + config.uncertainty_p_lon, config.p_lat_initial + config.uncertainty_p_lat)

    return tuple_vertices


def generate_tuples_vertices_polygons_initial(config: Configuration) -> Tuple:
    """
    Returns tuples of vertices for the initial polygons in two directions.
    """
    config = config.planning

    tuple_vertices_polygon_lon = (
        config.p_lon_initial - config.uncertainty_p_lon, config.v_lon_initial - config.uncertainty_v_lon,
        config.p_lon_initial + config.uncertainty_p_lon, config.v_lon_initial + config.uncertainty_v_lon)

    tuple_vertices_polygon_lat = (
        config.p_lat_initial - config.uncertainty_p_lat, config.v_lat_initial - config.uncertainty_v_lat,
        config.p_lat_initial + config.uncertainty_p_lat, config.v_lat_initial + config.uncertainty_v_lat)

    return tuple_vertices_polygon_lon, tuple_vertices_polygon_lat


def propagate_polygon(polygon: ReachPolygon, polygon_zero_state: ReachPolygon, dt: float,
                      v_min: float, v_max: float) -> ReachPolygon:
    """
    Propagates the (lon/lat) polygon of a reach node.

    Steps:
        1. convexify the polygon
        2. compute the linear mapping (zero-input response) of the polygon
        3. compute the minkowski sum of the zero-input and zero-state polygons
        4. intersect with halfspaces to consider velocity limits
    """
    polygon_processed = polygon.clone(convexify=True)
    polygon_processed = util_geometry.linear_mapping(polygon_processed, (1, dt, 0, 1))
    polygon_processed = util_geometry.minkowski_sum(polygon_processed, polygon_zero_state)
    polygon_processed = polygon_processed.intersect_halfspace(0, 1, v_max)
    polygon_processed = polygon_processed.intersect_halfspace(0, -1, -v_min)

    return polygon_processed


def project_propagated_sets_to_position_domain(list_propagated_sets: List[ReachNode]) -> List[ReachPolygon]:
    """
    Returns a list of rectangles projected onto the position domain.
    """
    return [propagated_set.position_rectangle for propagated_set in list_propagated_sets]


def create_repartitioned_rectangles(list_rectangles: List[ReachPolygon], size_grid: float) -> List[ReachPolygon]:
    """
    Returns a list of repartitioned rectangles.

    Steps:
        1. obtain the minimum lon/lat positions of the list of rectangles.
        2. discretize rectangles
        3. repartition the rectangles into a new list of rectangles.
        4. restore the rectangles back to undiscretized ones.
    """

    if not list_rectangles:
        return []

    tuple_p_min_rectangles = compute_minimum_positions_of_rectangles(list_rectangles)

    list_rectangles_discretized = discretize_rectangles(list_rectangles, tuple_p_min_rectangles, size_grid)

    list_rectangles_repartitioned = repartition_rectangles(list_rectangles_discretized)

    list_rectangles_undiscretized = undiscretized_rectangles(list_rectangles_repartitioned,
                                                             tuple_p_min_rectangles, size_grid)

    return list_rectangles_undiscretized


def compute_minimum_positions_of_rectangles(list_rectangles: List[ReachPolygon]) -> Tuple[float, float]:
    """
    Returns minimum lon/lat positions of the given list of rectangles.
    """
    p_lon_min_rectangles = min([rectangle.p_lon_min for rectangle in list_rectangles])
    p_lat_min_rectangles = min([rectangle.p_lat_min for rectangle in list_rectangles])

    return p_lon_min_rectangles, p_lat_min_rectangles


def compute_extremum_positions_of_rectangles(list_rectangles: List[ReachPolygon]) -> Tuple[float, float, float, float]:
    """
    Returns extremum lon/lat positions of the given list of rectangles.
    """
    p_lon_min_rectangles = min([rectangle.p_lon_min for rectangle in list_rectangles])
    p_lon_max_rectangles = max([rectangle.p_lon_max for rectangle in list_rectangles])
    p_lat_min_rectangles = min([rectangle.p_lat_min for rectangle in list_rectangles])
    p_lat_max_rectangles = max([rectangle.p_lat_max for rectangle in list_rectangles])

    return p_lon_min_rectangles, p_lon_max_rectangles, p_lat_min_rectangles, p_lat_max_rectangles


def discretize_rectangles(list_rectangles: List[ReachPolygon], tuple_p_min_rectangles: Tuple[float, float],
                          size_grid: float) -> List[ReachPolygon]:
    """
    Discretizes the given list of rectangles.

    p_discretized = (p_undiscretized - p_min) / size_grid
    For over-approximation, take floor for minimum values, and take ceil for maximum values.
    """
    list_rectangles_discretized = []
    p_lon_min_rectangles, p_lat_min_rectangles = tuple_p_min_rectangles
    p_lon_min_rectangles = Decimal(p_lon_min_rectangles)
    p_lat_min_rectangles = Decimal(p_lat_min_rectangles)
    size_grid = Decimal(size_grid)

    for rectangle in list_rectangles:
        p_lon_min = floor((Decimal(rectangle.p_lon_min) - p_lon_min_rectangles) / size_grid)
        p_lat_min = floor((Decimal(rectangle.p_lat_min) - p_lat_min_rectangles) / size_grid)
        p_lon_max = ceil((Decimal(rectangle.p_lon_max) - p_lon_min_rectangles) / size_grid)
        p_lat_max = ceil((Decimal(rectangle.p_lat_max) - p_lat_min_rectangles) / size_grid)

        list_rectangles_discretized.append(
            ReachPolygon.from_rectangle_vertices(p_lon_min, p_lat_min, p_lon_max, p_lat_max))

    return list_rectangles_discretized


def repartition_rectangles(list_rectangles: List[ReachPolygon]) -> List[ReachPolygon]:
    """
    Returns a list of repartitioned rectangles.

    Steps:
        1. Obtain a list of vertical segments representing the contour of the union of the input rectangles.
        2. Create repartitioned rectangles from the list of vertical segments using the sweep line algorithm.
    """
    list_segments_vertical = SweepLine.obtain_vertical_segments_from_rectangles(list_rectangles)

    list_rectangles_repartitioned = SweepLine.create_rectangles_from_vertical_segments(list_segments_vertical)

    return list_rectangles_repartitioned


def undiscretized_rectangles(list_rectangles_discretized: List[ReachPolygon],
                             tuple_p_min_rectangles: Tuple[float, float], size_grid: float) -> List[ReachPolygon]:
    """
    Restores previously discretized rectangles back to undiscretized ones.

    p_undiscretized = p_discretized * size_grid + p_min
    """
    list_rectangles_undiscretized = []
    p_lon_min_rectangles, p_lat_min_rectangles = tuple_p_min_rectangles

    for rectangle in list_rectangles_discretized:
        p_lon_min = rectangle.p_lon_min * size_grid + p_lon_min_rectangles
        p_lat_min = rectangle.p_lat_min * size_grid + p_lat_min_rectangles
        p_lon_max = rectangle.p_lon_max * size_grid + p_lon_min_rectangles
        p_lat_max = rectangle.p_lat_max * size_grid + p_lat_min_rectangles

        list_rectangles_undiscretized.append(
            ReachPolygon.from_rectangle_vertices(p_lon_min, p_lat_min, p_lon_max, p_lat_max))

    return list_rectangles_undiscretized


def check_collision_and_split_rectangles(collision_checker, step: int, list_rectangles: List[ReachPolygon],
                                         radius_terminal_split: float) -> List[ReachPolygon]:
    """
    Checks collision status of the input rectangles and split them if colliding.
    """
    list_rectangles_collision_free = []

    if not list_rectangles or not collision_checker:
        return []

    radius_terminal_squared = radius_terminal_split ** 2
    for rectangle in list_rectangles:
        list_rectangles_collision_free += create_collision_free_rectangles(step, collision_checker, rectangle,
                                                                           radius_terminal_squared)

    return list_rectangles_collision_free


def create_collision_free_rectangles(step: int, collision_checker, rectangle: ReachPolygon,
                                     radius_terminal_squared: float) -> List[ReachPolygon]:
    """
    Recursively creates a list of collision-free rectangles.

    If a collision happens between a rectangle and other object, and the diagonal of the rectangle is greater
    than the terminal radius, it is split into two new rectangles along its longer (lon/lat) edge.
    """
    # case 1: rectangle does not collide, return itself
    if not collision_checker.collides_at_step(step, rectangle):
        return [rectangle]

    # case 2: the diagonal is smaller than the terminal radius, return nothing
    elif rectangle.diagonal_squared < radius_terminal_squared:
        return []

    # case 3: colliding but diagonal is long enough. split into two new rectangles.
    else:
        rectangle_split_1, rectangle_split_2 = split_rectangle_into_two(rectangle)
        list_rectangles_split_1 = create_collision_free_rectangles(step, collision_checker, rectangle_split_1,
                                                                   radius_terminal_squared)
        list_rectangles_split_2 = create_collision_free_rectangles(step, collision_checker, rectangle_split_2,
                                                                   radius_terminal_squared)

        return list_rectangles_split_1 + list_rectangles_split_2


def split_rectangle_into_two(rectangle: ReachPolygon) -> Tuple[ReachPolygon, ReachPolygon]:
    """
    Returns two rectangles each of which is a half of the initial rectangle.

    Split in the longer axis (longitudinal / lateral or x / y).
    """
    if (rectangle.p_lon_max - rectangle.p_lon_min) > (rectangle.p_lat_max - rectangle.p_lat_min):
        rectangle_split_1 = ReachPolygon.from_rectangle_vertices(rectangle.p_lon_min, rectangle.p_lat_min,
                                                                 rectangle.p_lon_center, rectangle.p_lat_max)
        rectangle_split_2 = ReachPolygon.from_rectangle_vertices(rectangle.p_lon_center, rectangle.p_lat_min,
                                                                 rectangle.p_lon_max, rectangle.p_lat_max)
    else:
        rectangle_split_1 = ReachPolygon.from_rectangle_vertices(rectangle.p_lon_min, rectangle.p_lat_min,
                                                                 rectangle.p_lon_max, rectangle.p_lat_center)
        rectangle_split_2 = ReachPolygon.from_rectangle_vertices(rectangle.p_lon_min, rectangle.p_lat_center,
                                                                 rectangle.p_lon_max, rectangle.p_lat_max)

    return rectangle_split_1, rectangle_split_2


def construct_reach_nodes(drivable_area: List[ReachPolygon],
                          list_propagated_set: List[ReachNode],
                          has_multi_generation: bool = False) -> List[ReachNode]:
    """
    Constructs nodes of the reachability graph.

    The nodes are constructed by intersecting propagated sets with the drivable areas to determine the reachable
    positions and velocities.

    Steps:
        1. examine the adjacency of drivable areas and the propagated sets. They are considered adjacent if they
           overlap in the position domain.
        2. create a node from each drivable area and its adjacent propagated sets.
    """
    reachable_set = []

    list_rectangles_propagated_set = [propagated_set.position_rectangle for propagated_set in list_propagated_set]
    list_rectangles_drivable_area = drivable_area
    dict_rectangle_adjacency = util_geometry.create_adjacency_dictionary(list_rectangles_drivable_area,
                                                                         list_rectangles_propagated_set)

    for idx_drivable_area, list_idx_propagated_sets_adjacent in dict_rectangle_adjacency.items():
        rectangle_drivable_area = list_rectangles_drivable_area[idx_drivable_area]

        reach_node = construct_reach_node(rectangle_drivable_area, list_propagated_set,
                                          list_idx_propagated_sets_adjacent, has_multi_generation)
        if reach_node:
            reachable_set.append(reach_node)

    return reachable_set


def construct_reach_node(rectangle_drivable_area: ReachPolygon,
                         list_propagated_set: List[ReachNode],
                         list_idx_propagated_sets_adjacent: List[int],
                         multi_generation=False):
    """
    Returns a reach node constructed from the propagated sets.

    Iterate through base sets that are adjacent to the drivable areas, and intersect the base sets with position
    constraints from the drivable areas. A non-empty intersected polygon imply that it is a valid base set and is
    considered as a parent of the rectangle (reachable from the node from which the base set was propagated).
    """
    if not multi_generation:
        Node = ReachNode

    else:
        Node = ReachNodeMultiGeneration

    list_nodes_parent = []
    list_vertices_polygon_lon_new = []
    list_vertices_polygon_lat_new = []
    # iterate through adjacent propagated sets
    for idx_base_set_adjacent in list_idx_propagated_sets_adjacent:
        propagated_set_adjacent = list_propagated_set[idx_base_set_adjacent]
        polygon_lon = propagated_set_adjacent.polygon_lon
        polygon_lat = propagated_set_adjacent.polygon_lat
        # cut down to position range of the drivable area rectangle
        try:
            polygon_lon = polygon_lon.intersect_halfspace(1, 0, rectangle_drivable_area.p_lon_max)
            polygon_lon = polygon_lon.intersect_halfspace(-1, 0, -rectangle_drivable_area.p_lon_min)
            polygon_lat = polygon_lat.intersect_halfspace(1, 0, rectangle_drivable_area.p_lat_max)
            polygon_lat = polygon_lat.intersect_halfspace(-1, 0, -rectangle_drivable_area.p_lat_min)

        except AttributeError:
            pass

        else:
            # add to list if the intersected polygons are non-empty
            if polygon_lon and not polygon_lon.is_empty and polygon_lat and not polygon_lat.is_empty:
                list_vertices_polygon_lon_new += polygon_lon.vertices
                list_vertices_polygon_lat_new += polygon_lat.vertices
                node_parent = propagated_set_adjacent.source_propagation
                list_nodes_parent.append(node_parent)

    # if there is at least one valid propagated set, create node
    if list_vertices_polygon_lon_new and list_vertices_polygon_lat_new:
        polygon_lon_new = ReachPolygon.from_polygon(ReachPolygon(list_vertices_polygon_lon_new).convex_hull)
        polygon_lat_new = ReachPolygon.from_polygon(ReachPolygon(list_vertices_polygon_lat_new).convex_hull)
        reach_node = Node(polygon_lon_new, polygon_lat_new)
        reach_node.source_propagation = list_nodes_parent

        return reach_node


def connect_children_to_parents(step: int, list_nodes: List[ReachNode]):
    """
    Connects child reach nodes to their parent nodes.
    """
    list_nodes_reachable_set_new = []

    for node_child in list_nodes:
        node_child.step = step
        # update parent-child relationship
        for node_parent in node_child.source_propagation:
            node_child.add_parent_node(node_parent)
            node_parent.add_child_node(node_child)

        list_nodes_reachable_set_new.append(node_child)

    return list_nodes_reachable_set_new


def adapt_rectangles_to_grid(list_rectangles: List[ReachPolygon], size_grid: float) -> List[ReachPolygon]:
    """
    Adapts the given list of position rectangles to a Cartesian grid.
    """

    def is_disjoint(_rectangle: ReachPolygon, _cell: Cell) -> bool:
        """
        Returns True if the given rectangle and cell are disjoint.
        """
        if _rectangle.p_lon_max < _cell.x_min or _rectangle.p_lon_min > _cell.x_max or \
                _rectangle.p_lat_max < _cell.y_min or _rectangle.p_lat_min > _cell.y_max:
            return True

        return False

    def adapt_rectangle_to_cell(_rectangle: ReachPolygon, _cell: Cell) -> ReachPolygon:
        """
        Adapts the given rectangle to the given cell.
        """
        rectangle_intersected = _rectangle.clone(convexify=False)
        rectangle_intersected = rectangle_intersected.intersect_halfspace(1, 0, _cell.x_max)
        rectangle_intersected = rectangle_intersected.intersect_halfspace(-1, 0, -_cell.x_min)
        rectangle_intersected = rectangle_intersected.intersect_halfspace(0, 1, _cell.y_max)
        rectangle_intersected = rectangle_intersected.intersect_halfspace(0, -1, -_cell.y_min)

        return rectangle_intersected

    list_rectangles_adapted = []
    tuple_extremum = compute_extremum_positions_of_rectangles(list_rectangles)
    grid = Grid(*tuple_extremum, size_grid)

    for rectangle in list_rectangles:
        for cell in grid.list_cells:
            if not is_disjoint(rectangle, cell):
                list_rectangles_adapted.append(adapt_rectangle_to_cell(rectangle, cell))

    return list_rectangles_adapted


def remove_rectangles_out_of_kamms_circle(dt: float, a_max: float,
                                          list_rectangles: List[ReachPolygon]) -> List[ReachPolygon]:
    """
    Discards position rectangles that do not intersect with Kamm's friction circle.
    
    :param dt: time duration
    :param a_max: maximum acceleration
    :param list_rectangles: input list of rectangles
    """
    center_circle = (0, 0)
    radius_circle = 0.5 * a_max * dt ** 2

    list_idx_rectangles_to_be_deleted = list()
    for index, rectangle in enumerate(list_rectangles):
        if not util_geometry.rectangle_intersects_with_circle(rectangle, center_circle, radius_circle):
            list_idx_rectangles_to_be_deleted.append(index)

    return [rectangle for index, rectangle in enumerate(list_rectangles)
            if index not in list_idx_rectangles_to_be_deleted]


def compute_area_of_reach_nodes(list_nodes_reach: List[ReachNode]) -> float:
    """
    Computes the area of a given list of reach nodes.
    """
    area = 0.0
    if not list_nodes_reach:
        return area

    for node in list_nodes_reach:
        area += (node.p_lon_max - node.p_lon_min) * (node.p_lat_max - node.p_lat_min)
    return area


def connected_reachset_py(list_nodes_reach: List[ReachNode], num_digits: int):
    """
    Determines connected sets in the position domain.

    Returns a dictionary in the form of {node index:list of tuples (node index, node index)}.
    This function is the equivalent python function to pycrreach.connected_reachset_boost().
    """
    coefficient = np.power(10.0, num_digits)
    dict_adjacency = defaultdict(list)
    list_position_rectangles = list()
    # preprocess
    for node_reach in list_nodes_reach:
        # enlarge position rectangles
        vertices_rectangle_scaled = (np.floor(node_reach.p_lon_min * coefficient),
                                     np.floor(node_reach.p_lat_min * coefficient),
                                     np.ceil(node_reach.p_lon_max * coefficient),
                                     np.ceil(node_reach.p_lat_max * coefficient))
        list_position_rectangles.append(ReachPolygon.from_rectangle_vertices(*vertices_rectangle_scaled))

    # iterate over all rectangles
    for idx1, position_rect_1 in enumerate(list_position_rectangles):
        for idx2, position_rect_2 in enumerate(list_position_rectangles):
            if idx1 == idx2:
                continue

            # check for dict_adjacency via shapely intersects() function. If True, add tuple of idx to dict
            if position_rect_1.intersects(position_rect_2):
                dict_adjacency[idx1].append((idx1, idx2))

    return dict_adjacency


def lon_interval_connected_set(connected_set):
    """
    Projects a connected set onto longitudinal position domain and returns min/max longitudinal positions.
    """
    # get min and max values for each reachable set in the connected set
    min_max_array = np.asarray([[reach_node.p_lon_min, reach_node.p_lon_max] for reach_node in connected_set])

    # get minimum and maximum value for the connected set
    min_connected_set = np.min(min_max_array[:, 0])
    max_connected_set = np.max(min_max_array[:, 1])

    return min_connected_set, max_connected_set


def lat_interval_connected_set(connected_set):
    """
    Projects a connected set onto lateral position domain and returns min/max lateral positions.
    """
    # get min and max values for each reachable set in the connected set
    min_max_array = np.asarray([[reach_node.p_lat_min, reach_node.p_lat_max] for reach_node in connected_set])

    # get minimum and maximum value for the connected set
    min_connected_set = np.min(min_max_array[:, 0])
    max_connected_set = np.max(min_max_array[:, 1])

    return min_connected_set, max_connected_set


def lon_velocity_interval_connected_set(connected_set):
    """
    Projects a connected reachable set onto longitudinal velocity domain and returns min/max longitudinal velocities
    """
    # get min and max values for each reachable set in the connected set
    min_max_array = np.asarray([[reach_node.polygon_lon.v_min, reach_node.polygon_lon.v_max]
                                for reach_node in connected_set])

    # get minimum and maximum value for the connected set
    min_connected_set = np.min(min_max_array[:, 0])
    max_connected_set = np.max(min_max_array[:, 1])

    return min_connected_set, max_connected_set


def determine_overlapping_nodes_with_lon_pos(list_nodes_reach: List[Union[pycrreach.ReachNode, ReachNode]],
                                             p_lon: float) -> List[Union[pycrreach.ReachNode, ReachNode]]:
    """
    Projects reachset nodes onto longitudinal position domain and determines reachset nodes which contain a given
    longitudinal position
    """
    set_nodes_overlap = set()

    for node_reach in list_nodes_reach:
        if np.greater_equal(round(p_lon * 10.0 ** 2), np.floor(node_reach.p_lon_min * 10.0 ** 2)) and \
                np.greater_equal(np.ceil(node_reach.p_lon_max * 10.0 ** 2), round(p_lon * 10.0 ** 2)):
            set_nodes_overlap.add(node_reach)

    return list(set_nodes_overlap)


def determine_connected_components(list_nodes_reach, exclude_small_area: bool = False):
    """
    Determines and returns the connected reachable sets in the position domain.

    Connected components are sorted according to a heuristic (area of connected reachable sets).

    :param list_nodes_reach: list of reach nodes
    :param exclude_small_area: excludes connected components with an area smaller than the threshold and if there are
        more than 1 connected component at the current time step

    :return: list of connected reachable sets
    """
    from commonroad_reach.data_structure.reach.driving_corridor import ConnectedComponent

    num_digits = 2

    if type(list_nodes_reach[0]) == pycrreach.ReachNode:
        overlap = pycrreach.connected_reachset_boost(list_nodes_reach, num_digits)
    else:
        overlap = connected_reachset_py(list_nodes_reach, num_digits)

    # adjacency list: list with tuples, e.g., (0, 1) representing that node 0 and node 1 are connected
    adjacency = []
    for v in overlap.values():
        adjacency += v

    list_connected_component = list()
    # create graph with nodes = reach nodes and edges = adjacency status
    graph = nx.Graph()
    graph.add_nodes_from(list(range(len(list_nodes_reach))))
    graph.add_edges_from(adjacency)

    for set_indices_nodes_reach_connected in nx.connected_components(graph):
        list_nodes_reach_in_cc = [list_nodes_reach[idx] for idx in set_indices_nodes_reach_connected]
        connected_component = ConnectedComponent(list_nodes_reach_in_cc)

        # todo: add threshold to config?
        if exclude_small_area and len(set_indices_nodes_reach_connected) >= 2 and connected_component.area < 0.05:
            continue

        list_connected_component.append(connected_component)

    # sort connected components based on their areas
    list_connected_component.sort(key=lambda cc: cc.area, reverse=True)

    return list_connected_component
