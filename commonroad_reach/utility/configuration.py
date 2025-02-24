import os
from typing import Tuple, Dict
from collections import defaultdict

import commonroad_clcs.pycrccosy as pycrccosy
from commonroad_clcs.util import (
    compute_orientation_from_polyline,
    compute_pathlength_from_polyline
)
import numpy as np

np.seterr(divide='ignore', invalid='ignore')


def compute_disc_radius_and_distance(length: float, width: float, ref_point="CENTER", dist_axle_rear=None) \
        -> Tuple[float, float]:
    """
    Computes the radius of discs and their distances used as the approximation of the shape of the ego vehicle.

    .. note::
        Vehicle occupancy is approximated by three equally sized discs with equidistant center points.
        (see Ziegler, J. and Stiller, C. (2010) **"Fast collision checking for intelligent vehicle motion planning"**,
        IEEE IV

    :param length: vehicle length
    :param width: vehicle width
    :param ref_point: "CENTER" or "REAR"
    :param dist_axle_rear: if ref_point == "REAR", the distance between vehicle center and rear axle has to be provided
    :return: radius_disc: radius of discs
    :return: dist_circles: distance between the first and the third circle
    """
    assert length >= 0 and width >= 0, f"Invalid vehicle dimensions: length = {length}, width = {width}"

    if np.isclose(length, 0.0) and np.isclose(width, 0.0):
        return 0.0, 0.0

    # half width of the ego vehicle
    half_width = width / 2

    if ref_point == "CENTER":
        # second circle center point is exactly at the geometric center of vehicle model
        # the other circles are placed equidistant along the longitudinal axis
        half_length = (length / 3) / 2
        radius = (half_length ** 2 + half_width ** 2) ** 0.5

        # ceil up to 1 digit
        # radius_disc = np.ceil(radius * 10) / 10
        radius_disc = radius
        dist_circles = length / 3 * 2

    elif ref_point == "REAR":
        # first circle center point has to be exactly on rear axis position of vehicle model
        assert dist_axle_rear >= 0, f"Please provide a valid value for the rear axle distance (dist_axle_rear = " \
                                    f"{dist_axle_rear})"
        if dist_axle_rear < length / 3:
            half_length = length / 2 - dist_axle_rear
            radius = (half_length ** 2 + half_width ** 2) ** 0.5

            # ceil up to 1 digit
            # radius_disc = np.ceil(radius * 10) / 10
            radius_disc = radius
            dist_circles = dist_axle_rear * 2

        else:
            half_length = (length / 3) / 2 + (dist_axle_rear - length / 3)
            radius = (half_length ** 2 + half_width ** 2) ** 0.5

            radius_disc = radius
            dist_circles = dist_axle_rear * 2
    else:
        raise Exception("reference point has to be either 'CENTER' or 'REAR'")

    return radius_disc, dist_circles


def compute_disc_radius_and_wheelbase(length: float, width: float, wheelbase: float = None) -> Tuple[float, float]:
    """
    Computes the radius of the discs to approximate the shape of vehicle.
    
    If wheelbase is not given, it is assumed that the front and rear axles are positioned at length * 1/6
    and length * 5/6 of the shape, thus yielding a wheelbase of length * 4/6.

    :param length: vehicle length
    :param width: vehicle width
    :param wheelbase: wheelbase of the vehicle. If given, it is used to compute the radius of discs.
    :return: radius_disc: radius of discs
    :return: wheelbase: wheelbase computed from vehicle dimensions
    """
    assert length >= 0 and width >= 0, f"Invalid vehicle dimensions: length = {length}, width = {width}"

    if np.isclose(length, 0.0) and np.isclose(width, 0.0):
        return 0.0, 0.0

    # wheelbase is the distance between the front and rear axles
    wheelbase = wheelbase or (length / 3 * 2)

    length_square = (length / 3) / 2
    width_square = width / 2
    radius = (length_square ** 2 + width_square ** 2) ** 0.5

    radius_disc = radius

    return radius_disc, wheelbase


def compute_inflation_radius(mode_inflation: int, length: float, width: float, radius_disc: float) -> float:
    """
    Computes the radius to inflate the obstacles for collision check of the ego vehicle.

    Based on the specified mode, we obtain either under or over-approximation of the shape of the ego vehicle.

    :return: radius_inflation: radius of inflation
    """
    assert length >= 0 and width >= 0, f"Invalid vehicle dimensions: length = {length}, width = {width}"

    # Inscribed circle (under-approximation of the shape of the ego vehicle)
    if mode_inflation == 1:
        return length / 2 if length < width else width / 2

    # Circumscribed circle (over-approximation of the shape of the ego vehicle)
    elif mode_inflation == 2:
        return np.sqrt(length ** 2 + width ** 2) / 2

    # Three disc approximation of vehicle occupancy
    elif mode_inflation == 3:
        return radius_disc


def create_curvilinear_coordinate_system(reference_path: np.ndarray, limit_projection_domain: float = 30.0,
                                         eps: float = 0.1, eps2: float = 1e-4) -> pycrccosy.CurvilinearCoordinateSystem:
    """
    Creates a curvilinear coordinate system from the given reference path.
    """
    CLCS = pycrccosy.CurvilinearCoordinateSystem(reference_path, limit_projection_domain, eps, eps2)
    CLCS.compute_and_set_curvature()

    return CLCS


def compute_initial_state_cart(config) -> Tuple:
    """
    Computes the initial state of the ego vehicle given a planning problem in the Cartesian coordinate system.
    """
    planning_problem = config.planning_problem
    state_initial = planning_problem.initial_state
    wb_rear_axle = config.vehicle.ego.wb_rear_axle

    x, y = state_initial.position
    o = state_initial.orientation
    v = state_initial.velocity

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


def compute_initial_state_cvln(config) -> Tuple:
    """
    Computes the initial state of the ego vehicle given a planning problem or a state in a curvilinear coordinate system.

    .. note::
        For the transformation of the ego vehicle's velocity to the curvilinear coordinate system, it is assumed
        that d * kappa_ref << 1 holds, where d is the distance of the ego vehicle to the reference path and kappa_ref
        is the curvature of the reference path

    :param config: configuration file
    """
    planning_problem = config.planning_problem
    state_initial = planning_problem.initial_state
    wb_rear_axle = config.vehicle.ego.wb_rear_axle

    x, y = state_initial.position
    orientation = state_initial.orientation
    v = state_initial.velocity

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


def read_lut_longitudinal_enlargement(reference_point: str, dist_circles: float, path_to_lut: str) -> Dict:
    """
    Reads look-up table for longitudinal enlargement for collision checking in reachability analysis.

    :param reference_point
    :param dist_circles Distance between front and rear circle (see fun compute_disc_radius_and_distance() )
    :param path_to_lut: path where look-up table is stored
    :return: look-up table as dictionary
    """
    if reference_point == "REAR":
        base_name = 'lut_ref_rear_wheelbase_'

    elif reference_point == "CENTER":
        base_name = 'lut_wheelbase_'

    else:
        raise ValueError("Unknown reference point: {}".format(reference_point))

    table = np.loadtxt(os.path.join(path_to_lut, base_name + str(int(dist_circles * 10)) + '.txt'), skiprows=1,
                       delimiter=',')

    lut_lon = defaultdict(dict)
    # table entries: lateral position, min curvature, max curvature, enlargement
    for row in table:
        lut_lon[row[0]].update({tuple(row[1:3]): row[3]})

    return lut_lon
