import commonroad_dc.pycrccosy as pycrccosy
from math import ceil
import numpy as np

np.seterr(divide='ignore', invalid='ignore')


def compute_disc_radius_and_wheelbase(length, width, wheelbase: float = None):
    """Computes the radius of the discs to approximate the shape of vehicle.
    
    If wheelbase is not given, it is assumed that the the front and rear axles are positioned at length/6
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

    length_square = wheelbase / 4
    width_square = width / 2
    radius = (length_square ** 2 + width_square ** 2) ** 0.5

    # ceil up to 1 digit
    radius_disc = ceil(radius * 10) / 10

    return radius_disc, wheelbase


def create_curvilinear_coordinate_system(reference_path: np.ndarray, limit_projection_domain: float = 25.0,
                                         eps: float = 0.1) -> pycrccosy.CurvilinearCoordinateSystem:
    CLCS = pycrccosy.CurvilinearCoordinateSystem(reference_path, limit_projection_domain, eps)

    return CLCS


def compute_curvature_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """Computes the curvature of a given polyline

    Args:
        polyline (np.ndarray): polyline for the curvature computation

    Returns:
        np.ndarray: curvature of the polyline
    """
    assert isinstance(polyline, np.ndarray) and polyline.ndim == 2 and len(polyline[:, 0]) > 2, \
        "Polyline malformed for curvature computation p={}".format(polyline)

    x_d = np.gradient(polyline[:, 0])
    x_dd = np.gradient(x_d)
    y_d = np.gradient(polyline[:, 1])
    y_dd = np.gradient(y_d)

    # compute curvature
    curvature = (x_d * y_dd - x_dd * y_d) / ((x_d ** 2 + y_d ** 2) ** (3.0 / 2.0))

    return curvature


def compute_initial_state_CVLN(config):
    """Computes the initial state of the ego vehicle given a planning problem.

    For the transformation of the ego vehicle's velocity to the curvilinear coordinate system, it is assumed
    that d*kappa_ref << 1 holds, where d is the distance of the ego vehicle to the reference path and kappa_ref
    is the curvature of the reference path
    """
    planning_problem = config.planning_problem
    x, y = planning_problem.initial_state.position
    orientation = planning_problem.initial_state.orientation
    v = planning_problem.initial_state.velocity

    wheelbase = config.vehicle.ego.wheelbase

    if config.planning.reference_point == "CENTER":
        p_lon, p_lat = config.planning.CLCS.convert_to_curvilinear_coords(x, y)

    elif config.planning.reference_point == "REAR":
        p_lon, p_lat = config.planning.CLCS.convert_to_curvilinear_coords(
            x - wheelbase / 2 * np.cos(orientation), y - wheelbase / 2 * np.sin(orientation))
    else:
        raise Exception(f" Unknown reference point: {config.planning.reference_point}")

    reference_path = config.planning.reference_path
    ref_orientation = compute_orientation_from_polyline(reference_path)
    ref_path_length = compute_path_length_from_polyline(reference_path)
    orientation_interpolated = np.interp(p_lon, ref_path_length, ref_orientation)

    v_lon = v * np.cos(orientation - orientation_interpolated)
    v_lat = v * np.sin(orientation - orientation_interpolated)

    return (p_lon, p_lat), (v_lon, v_lat)


def compute_orientation_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """Computes the orientation of a given polyline"""
    assert isinstance(polyline, np.ndarray) and len(polyline) > 1 and polyline.ndim == 2 and len(polyline[0, :]) == 2, \
        "not a valid polyline. polyline = {}".format(polyline)

    if len(polyline) < 2:
        raise NameError("Cannot create orientation from polyline of length < 2")

    orientation = []
    for i in range(0, len(polyline) - 1):
        pt1 = polyline[i]
        pt2 = polyline[i + 1]
        tmp = pt2 - pt1
        orientation.append(np.arctan2(tmp[1], tmp[0]))

    for i in range(len(polyline) - 1, len(polyline)):
        pt1 = polyline[i - 1]
        pt2 = polyline[i]
        tmp = pt2 - pt1
        orientation.append(np.arctan2(tmp[1], tmp[0]))

    return np.array(orientation)


def compute_path_length_from_polyline(polyline: np.ndarray) -> np.ndarray:
    """Computes the path length of a given polyline"""
    assert isinstance(polyline, np.ndarray) and polyline.ndim == 2 and len(polyline[:, 0]) > 2, \
        "Polyline malformed for path length computation p={}".format(polyline)
    distance = [0]
    for i in range(1, len(polyline)):
        distance.append(distance[i - 1] + np.linalg.norm(polyline[i] - polyline[i - 1]))

    return np.array(distance)
