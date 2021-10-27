from math import ceil, floor
from typing import List, Tuple

from commonroad_reachset.data_structure.collision_checker import CollisionChecker
from commonroad_reachset.data_structure.configuration import Configuration
from commonroad_reachset.data_structure.reach.reach_node import ReachNode
from commonroad_reachset.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reachset.utility import geometry as util_geometry
from commonroad_reachset.utility.sweep_line import SweepLine


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
    # Step 1
    polygon_bounding = create_bounding_polygon(dt, a_min, a_max)

    # Step 2
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

    Args:
        polygon (ReachPolygon): (lon/lat) polygon to be propagated
        polygon_zero_state (ReachPolygon): (lon/lat) zero state polygon
        dt (float): dt of propagation
        v_min (float): (lon/lat) minimum velocity limit
        v_max (float): (lon/lat) maximum velocity limit
    """
    polygon_processed = polygon.clone(convexify=True)
    polygon_processed = util_geometry.linear_mapping(polygon_processed, (1, dt, 0, 1))
    polygon_processed = util_geometry.minkowski_sum(polygon_processed, polygon_zero_state)
    polygon_processed = polygon_processed.intersect_halfspace(0, 1, v_max)
    polygon_processed = polygon_processed.intersect_halfspace(0, -1, -v_min)

    return polygon_processed


def project_base_sets_to_position_domain(list_base_sets_propagated: List[ReachNode]) -> List[ReachPolygon]:
    """Returns a list of rectangles projected onto the position domain."""
    list_rectangles_projected = []
    for base_set in list_base_sets_propagated:
        p_lon_min, p_lon_max = base_set.polygon_lon.p_min, base_set.polygon_lon.p_max
        p_lat_min, p_lat_max = base_set.polygon_lat.p_min, base_set.polygon_lat.p_max
        list_rectangles_projected.append(
            ReachPolygon.from_rectangle_vertices(p_lon_min, p_lat_min, p_lon_max, p_lat_max))

    return list_rectangles_projected


def create_repartitioned_rectangles(list_rectangles: List[ReachPolygon], size_grid: float) -> List[ReachPolygon]:
    """Returns a list of repartitioned rectangles.

    Steps:
        1. obtain the minimum lon/lat positions of the list of rectangles.
        2. discretize rectangles
        3. repartition the rectangles into a new list of rectangles.
        4. restore the rectangles back to undiscretized ones.

    Args:
        list_rectangles (List[ReachPolygon]): the list of rectangles to be repartitioned
        size_grid (float, optional): grid size for axis alignment
    """

    if not list_rectangles:
        return []

    # Step 1
    tuple_p_min_rectangles = compute_minimum_positions_of_rectangles(list_rectangles)

    # Step 2
    list_rectangles_discretized = discretize_rectangles(list_rectangles, tuple_p_min_rectangles, size_grid)

    # Step 3
    list_rectangles_repartitioned = repartition_rectangles(list_rectangles_discretized)

    # Step 4
    list_rectangles_undiscretized = undiscretized_rectangles(list_rectangles_repartitioned,
                                                             tuple_p_min_rectangles, size_grid)

    return list_rectangles_undiscretized


def compute_minimum_positions_of_rectangles(list_rectangles: List[ReachPolygon]) -> Tuple[float, float]:
    """"Returns minimum lon/lat positions of the given list of rectangles."""
    p_lon_min_rectangles = min([rectangle.p_lon_min for rectangle in list_rectangles])
    p_lat_min_rectangles = min([rectangle.p_lat_min for rectangle in list_rectangles])

    return p_lon_min_rectangles, p_lat_min_rectangles


def discretize_rectangles(list_rectangles: List[ReachPolygon], tuple_p_min_rectangles: Tuple[float, float],
                          size_grid: float) -> List[ReachPolygon]:
    """Discretizes the given list of rectangles.

    p_discretized = (p_undiscretized - p_min) / size_grid
    get floor for min values, and get ceil for max values for over-approximation.
    """
    list_rectangles_discretized = []
    p_lon_min_rectangles, p_lat_min_rectangles = tuple_p_min_rectangles
    factor_scale = 10000

    size_grid = int(size_grid * factor_scale)
    p_lon_min_rectangles = int(p_lon_min_rectangles * factor_scale)
    p_lat_min_rectangles = int(p_lat_min_rectangles * factor_scale)

    for rectangle in list_rectangles:
        p_lon_min = floor((int(rectangle.p_lon_min * factor_scale) - p_lon_min_rectangles) / size_grid)
        p_lat_min = floor((int(rectangle.p_lat_min * factor_scale) - p_lat_min_rectangles) / size_grid)
        p_lon_max = ceil((int(rectangle.p_lon_max * factor_scale) - p_lon_min_rectangles) / size_grid)
        p_lat_max = ceil((int(rectangle.p_lat_max * factor_scale) - p_lat_min_rectangles) / size_grid)

        list_rectangles_discretized.append(
            ReachPolygon.from_rectangle_vertices(p_lon_min, p_lat_min, p_lon_max, p_lat_max))

    return list_rectangles_discretized


def repartition_rectangles(list_rectangles: List[ReachPolygon]) -> List[ReachPolygon]:
    """Returns a list of repartitioned rectangles.

    Steps:
        1. Obtain a list of vertical segments representing the contour of the union of the rectangles.
        2. Create repartitioned rectangles from the list of vertical segments using sweep line algorithm.
    """
    # Step 1
    list_segments_vertical = SweepLine.obtain_vertical_segments_from_rectangles(list_rectangles)

    # Step 2
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


def check_collision_and_split_rectangles(collision_checker: CollisionChecker,
                                         time_step: int,
                                         list_rectangles: List[ReachPolygon],
                                         radius_terminal_split: float) -> List[ReachPolygon]:
    """Check collision status of the rectangles and split them if colliding.

    Args:
        collision_checker (CollisionChecker): collision checker in CART/CLCS
        time_step (int): time step at which the collision is checked
        list_rectangles (List[ReachPolygon]): list of rectangles to be checked
        radius_terminal_split (float): radius to terminate splitting
    """
    list_rectangles_collision_free = []

    if not list_rectangles:
        return []

    # check collision for each input rectangle
    radius_terminal_squared = radius_terminal_split ** 2
    for rectangle in list_rectangles:
        list_rectangles_collision_free += create_collision_free_rectangles(time_step, collision_checker, rectangle,
                                                                           radius_terminal_squared)

    return list_rectangles_collision_free


def create_collision_free_rectangles(time_step: int, collision_checker: CollisionChecker, rectangle: ReachPolygon,
                                     radius_terminal_squared: float) -> List[ReachPolygon]:
    """Recursively creates a list of collision-free rectangles.

    If a collision happens between a rectangle and other object, and that the diagonal of the rectangle is greater
    than the terminal radius, it is split into two new rectangles in whichever edge (lon/lat) that is longer.

    Args:
        time_step:
        collision_checker (CollisionChecker): collision checker
        rectangle (ReachPolygon): rectangle under examination
        radius_terminal_squared (float): squared terminal radius. it is squared for computation efficiency.
    """
    # case 1: rectangle does not collide, return itself
    if not collision_checker.collides_at_time_step(time_step, rectangle):
        return [rectangle]

    # case 2: the diagonal is smaller than the terminal radius, return nothing
    elif rectangle.diagonal_squared < radius_terminal_squared:
        return []

    # case 3: colliding but diagonal is long enough. split into two halves.
    else:
        rectangle_split_1, rectangle_split_2 = split_rectangle_into_two(rectangle)
        list_rectangles_split_1 = create_collision_free_rectangles(time_step, collision_checker, rectangle_split_1,
                                                                   radius_terminal_squared)
        list_rectangles_split_2 = create_collision_free_rectangles(time_step, collision_checker, rectangle_split_2,
                                                                   radius_terminal_squared)

        return list_rectangles_split_1 + list_rectangles_split_2


def split_rectangle_into_two(rectangle: ReachPolygon) -> Tuple[ReachPolygon, ReachPolygon]:
    """Returns two rectangles each of which is a half of the initial rectangle.

    Split in the longer axis of the two (longitudinal / lateral).
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


def adapt_base_sets_to_drivable_area(drivable_area: List[ReachPolygon],
                                     list_base_sets_propagated: List[ReachNode]) -> List[ReachNode]:
    """Creates adapted base sets from the computed drivable area.

    The nodes of the reachable set of the current time step is later created
    from these adapted base sets.

    Steps:
        1. examine the adjacency of rectangles of drivable area and the
           propagated base sets. They are considered adjacent if they
           overlap in the position domain.
        2. create a adapted base set for each of the drivable area rectangle
    """
    reachable_base_set_time_current = []

    # Step 1
    list_rectangles_base_sets = [base_set.position_rectangle for base_set in list_base_sets_propagated]
    list_rectangles_drivable_area = drivable_area
    dict_rectangle_adjacency = util_geometry.create_adjacency_dictionary(list_rectangles_drivable_area,
                                                                         list_rectangles_base_sets)
    # Step 2
    for idx_drivable_area, list_idx_base_sets_adjacent in dict_rectangle_adjacency.items():
        # examine each drivable area rectangle
        rectangle_drivable_area = list_rectangles_drivable_area[idx_drivable_area]

        base_set_adapted = adapt_base_set_to_drivable_area(rectangle_drivable_area, list_base_sets_propagated,
                                                           list_idx_base_sets_adjacent)
        if base_set_adapted:
            reachable_base_set_time_current.append(base_set_adapted)

    return reachable_base_set_time_current


def adapt_base_set_to_drivable_area(rectangle_drivable_area: ReachPolygon,
                                    list_base_sets_propagated: List[ReachNode],
                                    list_idx_base_sets_adjacent: List[int]):
    """Returns adapted base set created from a rectangle of the drivable area.

    Iterate through base sets that are adjacent to the rectangle of drivable area under examination (overlaps
    in position domain), and cut the base sets down with position constraints from the rectangle. Non-empty
    intersected lon/lat polygons imply that this is a valid base set and are considered as a parent of the
    rectangle (reachable from the node from which the base set is propagated).

    Args:
        rectangle_drivable_area (ReachPolygon): rectangle under examination
        list_base_sets_propagated (List[ReachBaseSet]): list of base sets
        list_idx_base_sets_adjacent (List[int]): list of adjacent ones
    """
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
        base_set_adapted = ReachNode(polygon_lon_new, polygon_lat_new)
        base_set_adapted.source_propagation = list_base_sets_parent

        return base_set_adapted


def create_nodes_of_reachable_set(time_step: int, list_base_sets_adapted: List[ReachNode]):
    """Creates list of reachable set nodes of the current time step.

    Each adapted base set is converted into a reachable set node. Also, the parent-child relationship with
    the nodes of the last time step is updated.
    """
    list_nodes_reachable_set_new = []

    for node_child in list_base_sets_adapted:
        node_child.time_step = time_step
        # update parent-child relationship
        for node_parent in node_child.source_propagation:
            node_child.add_parent_node(node_parent)
            node_parent.add_child_node(node_child)

        list_nodes_reachable_set_new.append(node_child)

    return list_nodes_reachable_set_new
