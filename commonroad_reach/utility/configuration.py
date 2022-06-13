import os
from typing import Tuple
from collections import defaultdict
import commonroad_dc.pycrccosy as pycrccosy
from commonroad_dc.geometry.util import compute_orientation_from_polyline, compute_pathlength_from_polyline
import numpy as np

np.seterr(divide='ignore', invalid='ignore')


def compute_disc_radius_and_distance(length: float, width: float, ref_point="CENTER", rear_axle_dist=None) \
        -> Tuple[float, float]:
    """
    Vehicle occupancy is approximated by three equally sized discs with equidistant centerpoints.
    (see Ziegler, J. and Stiller, C. (2010) "Fast collision checking for intelligent vehicle motion planning", IEEE IV
    Returns radius of discs and distance between the first and third disc.
    :param length: vehicle length
    :param width: vehicle width
    :param ref_point: either "CENTER" or "REAR"
    :param rear_axle_dist: if ref_point == "REAR", the distance between vehicle center and rear axle has to be provided
    :return: radius_disc: radius of discs
    :return: circle_distance: distance between first and third circle
    """
    assert length >= 0 and width >= 0, f"Invalid vehicle dimensions: length = {length}, width = {width}"

    if np.isclose(length, 0.0) and np.isclose(width, 0.0):
        return 0.0, 0.0

    # half width of the ego vehicle
    width_square = width / 2

    if ref_point == "CENTER":
        # second circle center point is exactly at the geometric center of vehicle model
        # the other circles are placed equidistant along the longitudinal axis
        length_square = (length / 3)/2
        radius = (length_square ** 2 + width_square ** 2) ** 0.5

        # ceil up to 1 digit
        # radius_disc = np.ceil(radius * 10) / 10
        radius_disc = radius
        circle_distance = (length / 3 * 2)

    elif ref_point == "REAR":
        # first circle center point has to be exactly on rear axis position of vehicle model
        assert rear_axle_dist >= 0, f"Please provide a valid value for the rear axle distance (rear_axle_distance = " \
                                    f"{rear_axle_dist})"
        if rear_axle_dist < length/3:
            length_square = length/2 - rear_axle_dist
            radius = (length_square ** 2 + width_square ** 2) ** 0.5

            # ceil up to 1 digit
            # radius_disc = np.ceil(radius * 10) / 10
            radius_disc = radius
            circle_distance = rear_axle_dist * 2
        else:
            length_square = (length / 3)/2 + (rear_axle_dist - length/3)
            radius = (length_square ** 2 + width_square ** 2) ** 0.5

            radius_disc = radius
            circle_distance = rear_axle_dist * 2
    else:
        raise Exception("reference point has to be either CENTER or REAR")

    return radius_disc, circle_distance


def compute_disc_radius_and_wheelbase(length: float, width: float, wheelbase: float = None):
    """Computes the radius of the discs to approximate the shape of vehicle.
    
    If wheelbase is not given, it is assumed that the front and rear axles are positioned at length/6
    and length * 5/6 of the shape, thus yielding a wheelbase of 4*length/6.

    Args:
        width:
        length:
        wheelbase ([float], optional): wheelbase of the vehicle. If the
        wheelbase is given, it is used to compute the radius of discs.
    """

    assert length >= 0 and width >= 0, f"Invalid vehicle dimensions: length = {length}, width = {width}"

    if np.isclose(length, 0.0) and np.isclose(width, 0.0):
        return 0.0, 0.0

    # wheelbase is the distance between the front and rear axles
    wheelbase = wheelbase or (length / 3 * 2)

    length_square = (length / 3)/2
    width_square = width / 2
    radius = (length_square ** 2 + width_square ** 2) ** 0.5

    # ceil up to 1 digit
    # radius_disc = np.ceil(radius * 10) / 10
    radius_disc = radius

    return radius_disc, wheelbase


def compute_inflation_radius(mode_inflation: int, length: float, width: float, radius_disc: float):
    """Computes the radius to inflate the obstacles for collision check.

    Based on the specified mode, we obtain either under- or over-approximation of the shape of the ego vehicle.
    """
    assert length >= 0 and width >= 0, f"Invalid vehicle dimensions: length = {length}, width = {width}"

    # Inscribed circle (under-approximation of the shape of the ego vehicle)
    if mode_inflation == 1:
        return length / 2 if length < width else width / 2

    # 2: Circumscribed circle (over-approximation of the shape of the ego vehicle)
    elif mode_inflation == 2:
        return length / 2 if length > width else width / 2

    # 3: Three disc approximation of vehicle occupancy
    elif mode_inflation == 3:
        return radius_disc


def create_curvilinear_coordinate_system(reference_path: np.ndarray, limit_projection_domain: float = 25.0,
                                         eps: float = 0.1) -> pycrccosy.CurvilinearCoordinateSystem:
    # create new Curvilinear Coordinate System and set curvature values for reference path
    CLCS = pycrccosy.CurvilinearCoordinateSystem(reference_path, limit_projection_domain, eps)
    CLCS.compute_and_set_curvature()

    return CLCS


def compute_initial_state_cart(config):
    """Computes the initial Cartesian state of the ego vehicle given a planning problem."""
    planning_problem = config.planning_problem
    wb_rear_axle = config.vehicle.ego.wb_rear_axle

    x, y = planning_problem.initial_state.position
    o = planning_problem.initial_state.orientation
    v = planning_problem.initial_state.velocity

    if config.planning.reference_point == "CENTER":
        pass

    elif config.planning.reference_point == "REAR":
        x = x - wb_rear_axle * np.cos(o)
        y = y - wb_rear_axle * np.sin(o)

    else:
        raise Exception(f"Unknown reference point: {config.planning.reference_point}")

    v_x = v * np.cos(o)
    v_y = v * np.sin(o)

    return (x, y), (v_x, v_y), o


def compute_initial_state_cvln(config):
    """Computes the initial curvilinear state of the ego vehicle given a planning problem.

    For the transformation of the ego vehicle's velocity to the curvilinear coordinate system, it is assumed
    that d*kappa_ref << 1 holds, where d is the distance of the ego vehicle to the reference path and kappa_ref
    is the curvature of the reference path
    """
    planning_problem = config.planning_problem
    wb_rear_axle = config.vehicle.ego.wb_rear_axle

    x, y = planning_problem.initial_state.position
    orientation = planning_problem.initial_state.orientation
    v = planning_problem.initial_state.velocity

    if config.planning.reference_point == "CENTER":
        p_lon, p_lat = config.planning.CLCS.convert_to_curvilinear_coords(x, y)

    elif config.planning.reference_point == "REAR":
        p_lon, p_lat = config.planning.CLCS.convert_to_curvilinear_coords(
            x - wb_rear_axle * np.cos(orientation), y - wb_rear_axle * np.sin(orientation))
    else:
        raise Exception(f"Unknown reference point: {config.planning.reference_point}")

    reference_path = config.planning.reference_path
    ref_orientation = compute_orientation_from_polyline(reference_path)
    ref_path_length = compute_pathlength_from_polyline(reference_path)
    orientation_interpolated = np.interp(p_lon, ref_path_length, ref_orientation)

    v_lon = v * np.cos(orientation - orientation_interpolated)
    v_lat = v * np.sin(orientation - orientation_interpolated)

    return (p_lon, p_lat), (v_lon, v_lat)


#TODO rename wheelbase parameter to circle distance
def read_lut_longitudinal_enlargement(reference_point: str, wheelbase: float, path_to_lut: str) -> dict:
    """
    Reads look-up table for longitudinal enlargement for collision checking in the reachability analysis.
    :param reference_point
    :param wheelbase
    :param path_to_lut: path where look-up table is stored
    :return: look-up table as dictionary
    """
    if reference_point == "REAR":
        base_name = 'lut_ref_rear_wheelbase_'
    elif reference_point == "CENTER":
        base_name = 'lut_wheelbase_'
    else:
        raise ValueError("<read_lut_longitudinal_enlargement>: unknown reference point: {}".format(reference_point))

    table = np.loadtxt(os.path.join(path_to_lut, base_name + str(int(wheelbase * 10)) + '.txt'), skiprows=1, delimiter=',')

    lut_lon = defaultdict(dict)
    # table entries: lateral position, min curvature, max curvature, enlargement
    for row in table:
        lut_lon[row[0]].update({tuple(row[1:3]): row[3]})
    return lut_lon
