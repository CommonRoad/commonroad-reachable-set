import logging
from decimal import Decimal

logger = logging.getLogger(__name__)
from math import ceil, floor
from typing import List, Tuple

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.grid import Grid, Cell
from commonroad_reach.data_structure.reach.reach_node import ReachNode, ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import geometry as util_geometry
from commonroad_reach.utility.sweep_line import SweepLine


def create_zero_state_polygon(dt: float, a_min: float, a_max: float) -> ReachPolygon:
    """Returns the zero-state polygon of the system.

    Steps:
        1. prepare a bounding polygon to be intersected with halfspaces.
        2. compute coefficients of halfspaces and intersect them with the bounding polygon. We use three
        halfspaces to approximate the upper bound of the polygon, this applies to the lower bound as well.
        A halfspace can be specified given a switching time (gamma).
    Args:
        dt (float): time duration (one step)
        a_min (float): minimum acceleration (maximum deceleration)
        a_max (float): maximum acceleration
    """
    polygon_bounding = create_bounding_polygon(dt, a_min, a_max)

    polygon_intersected = polygon_bounding
    for gamma in [0, 0.5, 1]:
        tuple_coefficients_upper, tuple_coefficients_lower = compute_halfspace_coefficients(dt, a_min, a_max, gamma)

        a, b, c = tuple_coefficients_upper
        polygon_intersected = polygon_intersected.intersect_halfspace(a, b, c)

        a, b, c = tuple_coefficients_lower
        polygon_intersected = polygon_intersected.intersect_halfspace(a, b, c)

    return ReachPolygon.from_polygon(polygon_intersected)


def create_bounding_polygon(dt: float, a_min: float, a_max: float) -> ReachPolygon:
    """Returns a polygon that has the absolute min/max reachable position and velocity as its bounds.

    Args:
        dt (float): time duration (one step)
        a_min (float): minimum acceleration (maximum deceleration)
        a_max (float): maximum acceleration
    """
    tuple_vertices = (0.5 * a_min * (dt ** 2), a_min * dt, 0.5 * a_max * (dt ** 2), a_max * dt)

    return ReachPolygon.from_rectangle_vertices(*tuple_vertices)


def compute_halfspace_coefficients(dt: float, a_min: float, a_max: float, gamma: float) -> Tuple:
    """Computes the coefficients of halfspaces to be intersected.

    Each halfspace has the form of ax + bv <= c. In the x-v plane, the x and v values are both dependent
    on gamma (switching time, see Söntges T-ITS 2018, Sec. III.A). Given a gamma, the maximum and minimum
    reachable x and v can be computed, respectively. The halfspace crossing the point x_ and v_ can be
    obtained through:
        v - v_ = (dv_/dx_) * (x - x_) = (dv_/dgamma * dgamma/dx_) * (x - x_)
        ==>
        (dv_/dgamma) * x + (-dx_/dgamma) * v <= (dv_/dgamma * x_ - -dx_/dgamma * v_)

    Essentially, the expanded form is performing the following computation:
    # 1. full braking (or acceleration) before the switching time
    x_0 = 0
    v_0 = 0
    a = a_min (a_max)
    t = gamma * dt - 0

    x_t = 0.5 * a * t**2 + v_0 * t + x_0
    v_t = a * t + v_0

    # 2. full acceleration (or braking) after the switching time
    x_0 = x_t
    v_0 = v_t
    a = a_max (a_min)
    t = dt - gamma * dt

    x_f = 0.5 * a * t**2 + v_0 * t + x_0
    v_f = a * t + v_0

    dx_XX & dv_XX are derivatives with regard to gamma.

    Args:
        dt (float): time duration
        a_min (float): a_min
        a_max (float): a_max
        gamma (float): switching time between full braking and acceleration
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
    """Returns tuple of vertices for the position rectangle construction."""
    config = config.planning

    tuple_vertices = (config.p_lon_initial - config.uncertainty_p_lon, config.p_lat_initial - config.uncertainty_p_lat,
                      config.p_lon_initial + config.uncertainty_p_lon, config.p_lat_initial + config.uncertainty_p_lat)

    return tuple_vertices


def generate_tuples_vertices_polygons_initial(config: Configuration) -> Tuple:
    """Returns tuples of vertices for the initial polygons in two directions."""
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
    """Propagates the polygon of a reachset node.

    Steps:
        1. clone and convexify the polygon
        2. compute the linear mapping (zero-input response) of the polygon
        3. compute the minkowski sum of the zero-input and zero-state polygons
        4. intersect with halfspaces to incorporate velocity limits
    """
    polygon_processed = polygon.clone(convexify=True)
    polygon_processed = util_geometry.linear_mapping(polygon_processed, (1, dt, 0, 1))
    polygon_processed = util_geometry.minkowski_sum(polygon_processed, polygon_zero_state)
    polygon_processed = polygon_processed.intersect_halfspace(0, 1, v_max)
    polygon_processed = polygon_processed.intersect_halfspace(0, -1, -v_min)

    return polygon_processed


def project_base_sets_to_position_domain(list_base_sets_propagated: List[ReachNode]) -> List[ReachPolygon]:
    """Returns a list of rectangles projected onto the position domain."""
    return [base_set.position_rectangle for base_set in list_base_sets_propagated]


def create_repartitioned_rectangles(list_rectangles: List[ReachPolygon], size_grid: float) -> List[ReachPolygon]:
    """Returns a list of repartitioned rectangles.

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
    """Returns minimum lon/lat positions of the given list of rectangles."""
    p_lon_min_rectangles = min([rectangle.p_lon_min for rectangle in list_rectangles])
    p_lat_min_rectangles = min([rectangle.p_lat_min for rectangle in list_rectangles])

    return p_lon_min_rectangles, p_lat_min_rectangles


def compute_extremum_positions_of_rectangles(list_rectangles: List[ReachPolygon]) -> Tuple[float, float, float, float]:
    """Returns extremum lon/lat positions of the given list of rectangles."""
    p_lon_min_rectangles = min([rectangle.p_lon_min for rectangle in list_rectangles])
    p_lon_max_rectangles = max([rectangle.p_lon_max for rectangle in list_rectangles])
    p_lat_min_rectangles = min([rectangle.p_lat_min for rectangle in list_rectangles])
    p_lat_max_rectangles = max([rectangle.p_lat_max for rectangle in list_rectangles])

    return p_lon_min_rectangles, p_lon_max_rectangles, p_lat_min_rectangles, p_lat_max_rectangles


def discretize_rectangles(list_rectangles: List[ReachPolygon], tuple_p_min_rectangles: Tuple[float, float],
                          size_grid: float) -> List[ReachPolygon]:
    """Discretizes the given list of rectangles.

    p_discretized = (p_undiscretized - p_min) / size_grid
    get floor for min values, and get ceil for max values for over-approximation.
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
    """Returns a list of repartitioned rectangles.

    Steps:
        1. Obtain a list of vertical segments representing the contour of the union of the rectangles.
        2. Create repartitioned rectangles from the list of vertical segments using sweep line algorithm.
    """
    list_segments_vertical = SweepLine.obtain_vertical_segments_from_rectangles(list_rectangles)

    list_rectangles_repartitioned = SweepLine.create_rectangles_from_vertical_segments(list_segments_vertical)

    return list_rectangles_repartitioned


def undiscretized_rectangles(list_rectangles_discretized: List[ReachPolygon],
                             tuple_p_min_rectangles: Tuple[float, float], size_grid: float) -> List[ReachPolygon]:
    """Restores previously discretized rectangles back to undiscretized.

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
    """Check collision status of the rectangles and split them if they are colliding."""
    list_rectangles_collision_free = []

    if not list_rectangles:
        return []

    if not collision_checker:
        return list_rectangles

    # check collision for each input rectangle
    radius_terminal_squared = radius_terminal_split ** 2
    for rectangle in list_rectangles:
        list_rectangles_collision_free += create_collision_free_rectangles(step, collision_checker, rectangle,
                                                                           radius_terminal_squared)

    return list_rectangles_collision_free


def create_collision_free_rectangles(step: int, collision_checker, rectangle: ReachPolygon,
                                     radius_terminal_squared: float) -> List[ReachPolygon]:
    """Recursively creates a list of collision-free rectangles.

    If a collision happens between a rectangle and other object, and that the diagonal of the rectangle is greater
    than the terminal radius, it is split into two new rectangles in whichever edge (lon/lat) that is longer.
    """
    # case 1: rectangle does not collide, return itself
    if not collision_checker.collides_at_step(step, rectangle):
        return [rectangle]

    # case 2: the diagonal is smaller than the terminal radius, return nothing
    elif rectangle.diagonal_squared < radius_terminal_squared:
        return []

    # case 3: colliding but diagonal is long enough. split into two halves.
    else:
        rectangle_split_1, rectangle_split_2 = split_rectangle_into_two(rectangle)
        list_rectangles_split_1 = create_collision_free_rectangles(step, collision_checker, rectangle_split_1,
                                                                   radius_terminal_squared)
        list_rectangles_split_2 = create_collision_free_rectangles(step, collision_checker, rectangle_split_2,
                                                                   radius_terminal_squared)

        return list_rectangles_split_1 + list_rectangles_split_2


def split_rectangle_into_two(rectangle: ReachPolygon) -> Tuple[ReachPolygon, ReachPolygon]:
    """Returns two rectangles each of which is a half of the initial rectangle.

    Split in the longer axis of the two (longitudinal / lateral or x / y).
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
                          list_base_sets_propagated: List[ReachNode],
                          has_multi_generation: bool = False) -> List[ReachNode]:
    """Constructs nodes of the reachability graph.

    The nodes are constructed by cutting down the propagated base sets to the drivable area to determine the reachable
    positions and velocities.

    Steps:
        1. examine the adjacency of drivable area and the propagated base sets. They are considered adjacent if they
           overlap in the position domain.
        2. create a node from each drivable area and its adjacent propagated base sets.
    """
    reachable_set = []

    list_rectangles_base_sets = [base_set.position_rectangle for base_set in list_base_sets_propagated]
    list_rectangles_drivable_area = drivable_area
    dict_rectangle_adjacency = util_geometry.create_adjacency_dictionary(list_rectangles_drivable_area,
                                                                         list_rectangles_base_sets)

    for idx_drivable_area, list_idx_base_sets_adjacent in dict_rectangle_adjacency.items():
        # examine each drivable area rectangle
        rectangle_drivable_area = list_rectangles_drivable_area[idx_drivable_area]

        reach_node = construct_reach_node(rectangle_drivable_area, list_base_sets_propagated,
                                                list_idx_base_sets_adjacent, has_multi_generation)
        if reach_node:
            reachable_set.append(reach_node)

    return reachable_set


def construct_reach_node(rectangle_drivable_area: ReachPolygon,
                         list_base_sets_propagated: List[ReachNode],
                         list_idx_base_sets_adjacent: List[int],
                         multi_generation=False):
    """Returns a reach node constructed from the propagated base sets.

    Iterate through base sets that are adjacent to the drivable area, and cut the base sets down with position
    constraints from the drivable area. A non-empty intersected polygon imply that it is a valid base set and is
    considered as a parent of the rectangle (reachable from the node from which the base set is propagated).
    """
    if not multi_generation:
        Node = ReachNode

    else:
        Node = ReachNodeMultiGeneration

    list_base_sets_parent = []
    list_vertices_polygon_lon_new = []
    list_vertices_polygon_lat_new = []
    # retrieve each of the adjacent base sets
    for idx_base_set_adjacent in list_idx_base_sets_adjacent:
        base_set_adjacent = list_base_sets_propagated[idx_base_set_adjacent]
        polygon_lon = ReachPolygon.from_polygon(base_set_adjacent.polygon_lon)
        polygon_lat = ReachPolygon.from_polygon(base_set_adjacent.polygon_lat)
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
                list_base_sets_parent.append(base_set_adjacent.source_propagation)

    # if there is at least one valid base set, create the adapted base set
    if list_vertices_polygon_lon_new and list_vertices_polygon_lat_new:
        polygon_lon_new = ReachPolygon.from_polygon(ReachPolygon(list_vertices_polygon_lon_new).convex_hull)
        polygon_lat_new = ReachPolygon.from_polygon(ReachPolygon(list_vertices_polygon_lat_new).convex_hull)
        reach_node = Node(polygon_lon_new, polygon_lat_new)
        reach_node.source_propagation = list_base_sets_parent

        return reach_node


def connect_children_to_parents(step: int, list_nodes: List[ReachNode]):
    """Connects the child reach nodes to their parent nodes."""
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
    """Adapts the given list of position rectangles to a Cartesian grid."""

    def is_disjoint(_rectangle: ReachPolygon, _cell: Cell) -> bool:
        """Returns True if the given rectangle and cell are disjoint."""
        if _rectangle.p_lon_max < _cell.x_min or _rectangle.p_lon_min > _cell.x_max or \
                _rectangle.p_lat_max < _cell.y_min or _rectangle.p_lat_min > _cell.y_max:
            return True

        return False

    def adapt_rectangle_to_cell(_rectangle: ReachPolygon, _cell: Cell) -> ReachPolygon:
        """Adapts the given rectangle to the given cell."""
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


def remove_rectangles_out_of_kamms_circle(time_duration: float, a_max: float,
                                          list_rectangles_adapted: List[ReachPolygon]) -> List[ReachPolygon]:
    """Discard position rectangles that do not intersect with Kamm's friction circle."""
    center_circle = (0, 0)
    radius_circle = 0.5 * a_max * time_duration ** 2

    list_idx_rectangles_to_be_deleted = list()
    for index, rectangle in enumerate(list_rectangles_adapted):
        if not util_geometry.rectangle_intersects_with_circle(rectangle, center_circle, radius_circle):
            list_idx_rectangles_to_be_deleted.append(index)

    return [rectangle for index, rectangle in enumerate(list_rectangles_adapted)
            if index not in list_idx_rectangles_to_be_deleted]
