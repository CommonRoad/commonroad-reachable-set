from typing import Union, List, Tuple, Dict, Optional

from commonroad.scenario.lanelet import Lanelet, LaneletType
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle
from commonroad_reachset.common.data_structure.configuration import Configuration
from commonroad_reachset.common.data_structure.road_network import RoadNetwork
from commonroad_reachset.common.data_structure.vehicle import Vehicle
from crmonitor.common.vehicle import VehicleClassification, StateLongitudinal, StateLateral


def create_vehicle_from_obstacle(config: Configuration, road_network: RoadNetwork,
                                 obstacle: Union[StaticObstacle, DynamicObstacle],
                                 dict_spot_prediction: Dict = None) -> Optional[Vehicle]:
    """
    Transforms a CommonRoad obstacle to a vehicle object

    :param config: configuration object
    :param road_network: road network
    :param obstacle: CommonRoad obstacle
    :param dict_spot_prediction: spot prediction of the obstacle over time
    :return: vehicle object
    """
    # todo: this might have to be changed for crossing vehicles
    vehicle_classification = VehicleClassification.ADJACENT_VEHICLE
    dt = config.planning.dt
    state_init = obstacle.initial_state
    dict_time_to_spot_prediction_occupancy = {}
    dict_time_to_spot_extrema = {}
    use_spot = config.planning.use_spot

    if isinstance(obstacle, StaticObstacle):
        acceleration = jerk = 0
        state_init.velocity = 0
        use_spot = False

    elif isinstance(obstacle, DynamicObstacle):
        acceleration = compute_acceleration(state_init.velocity,
                                            obstacle.prediction.trajectory.state_list[0].velocity, dt)
        jerk = compute_jerk(acceleration, 0, dt)

        dict_spot_prediction = dict_spot_prediction[obstacle.obstacle_id]
        # occupancy_predicted is a list of tuples of (time_step, (time_step, polygon)
        dict_time_to_spot_prediction_occupancy = {0: dict_spot_prediction["occupancy_predicted"][0][1]}

    else:
        raise Exception("Conversion from obstacle to Vehicle failed.")

    [[list_ids_lanelets_initial]] = [road_network.lanelet_network.find_lanelet_by_position([state_init.position])]
    list_lanelets_initial = [road_network.lanelet_network.find_lanelet_by_id(lanelet_id)
                             for lanelet_id in list_ids_lanelets_initial]

    if LaneletType.ACCESS_RAMP in list_lanelets_initial[0].lanelet_type:
        main_carriage_way_lanelet_id = find_main_carriage_way_lanelet_id(list_lanelets_initial[0],
                                                                         road_network.lanelet_network)
        lane = road_network.find_lane_by_obstacle([main_carriage_way_lanelet_id], [])

    else:
        lane = road_network.find_lane_by_obstacle(list_ids_lanelets_initial, list_ids_lanelets_initial)

    # convert initial state
    state_lon, state_lat = convert_to_curvilinear_state(state_init.position, state_init.velocity,
                                                        acceleration, jerk, state_init.orientation, config)
    dict_time_to_state_lon = {0: state_lon}
    dict_time_to_state_lat = {0: state_lat}
    dict_time_to_state_CR = {0: state_init}
    dict_time_to_state_signal = {0: obstacle.initial_signal_state}
    dict_time_to_lanelet_assignment = {0: list_ids_lanelets_initial}
    dict_time_to_classification = {0: vehicle_classification}

    # convert the rest of the states
    if isinstance(obstacle, StaticObstacle):
        for time_step in range(config.planning.time_steps_computation + 1):
            dict_time_to_state_lon[time_step] = state_lon
            dict_time_to_state_lat[time_step] = state_lat
            dict_time_to_state_CR[time_step] = state_init
            dict_time_to_state_signal[time_step] = obstacle.signal_state_at_time_step(time_step)
            [lanelet_assignment_at_time_step] = \
                road_network.lanelet_network.find_lanelet_by_position([state_init.position])
            dict_time_to_lanelet_assignment[time_step] = lanelet_assignment_at_time_step
            dict_time_to_classification[time_step] = vehicle_classification

    else:
        for state in obstacle.prediction.trajectory.state_list:
            if state_lon is None or state_lat is None:
                continue

            acceleration = compute_acceleration(state_lon.v, state.velocity, dt)
            jerk = compute_jerk(acceleration, 0, dt)

            state_lon_temp, state_lat_temp = convert_to_curvilinear_state(state.position, state.velocity, acceleration,
                                                                          jerk,
                                                                          state.orientation, config)
            if state_lon_temp is None or state_lat_temp is None:
                continue
            else:
                state_lon = state_lon_temp
                state_lat = state_lat_temp

            dict_time_to_state_lon[state.time_step] = state_lon
            dict_time_to_state_lat[state.time_step] = state_lat
            dict_time_to_state_CR[state.time_step] = state
            dict_time_to_state_signal[state.time_step] = obstacle.signal_state_at_time_step(state.time_step)
            [lanelet_assignment_at_time_step] = road_network.lanelet_network.find_lanelet_by_position(
                [state.position])
            dict_time_to_lanelet_assignment[state.time_step] = lanelet_assignment_at_time_step
            dict_time_to_classification[state.time_step] = vehicle_classification

            try:
                dict_time_to_spot_prediction_occupancy[state.time_step] = \
                    dict_spot_prediction["occupancy_predicted"][state.time_step][1]

            except IndexError:
                pass

    # obtain extrema of spot polygon
    for time_step, polygon in dict_time_to_spot_prediction_occupancy.items():
        list_p_lon = []
        list_p_lat = []

        for shape in polygon.shape.shapes:
            for x, y in shape.vertices:
                try:
                    p_lon, p_lat = config.planning.CLCS.convert_to_curvilinear_coords(x, y)

                except ValueError:
                    continue

                else:
                    list_p_lon.append(p_lon)
                    list_p_lat.append(p_lat)

            p_lon_min = min(list_p_lon) if len(list_p_lon) else None
            p_lon_max = max(list_p_lon) if len(list_p_lon) else None
            p_lat_min = min(list_p_lat) if len(list_p_lat) else None
            p_lat_max = max(list_p_lat) if len(list_p_lat) else None
            dict_time_to_spot_extrema[time_step] = (p_lon_min, p_lat_min, p_lon_max, p_lat_max)

    # create vehicle object
    vehicle = Vehicle(dict_time_to_state_lon, dict_time_to_state_lat, obstacle.obstacle_shape,
                      dict_time_to_state_CR, obstacle.obstacle_id, obstacle.obstacle_type,
                      dict_time_to_lanelet_assignment, dict_time_to_state_signal, dict_time_to_classification, lane,
                      dict_time_to_spot_prediction_occupancy, dict_time_to_spot_extrema, use_spot)
    return vehicle


def compute_acceleration(previous_velocity: float, current_velocity: float, dt: float):
    """
    Computes acceleration given velocity

    :param current_velocity: velocity of current time step
    :param previous_velocity: velocity of previous time step
    :param dt: time step size
    :return: acceleration
    """
    acceleration = (current_velocity - previous_velocity) / dt
    return acceleration


def compute_jerk(current_acceleration: float, previous_acceleration: float, dt: float) -> float:
    """
    Computes jerk given acceleration

    :param current_acceleration: acceleration of current time step
    :param previous_acceleration: acceleration of previous time step
    :param dt: time step size
    :return: jerk
    """
    jerk = (current_acceleration - previous_acceleration) / dt
    return jerk


def find_main_carriage_way_lanelet_id(lanelet: Lanelet, lanelet_network) -> int:
    """
    Searches for an adjacent lanelet part of the main carriageway

    :param lanelet_network:
    :param lanelet: start lanelet
    :return: ID of a lanelet which is part of the main carriageway
    """
    current_lanelet = lanelet
    if LaneletType.MAIN_CARRIAGE_WAY in current_lanelet.lanelet_type:
        return current_lanelet.lanelet_id
    while current_lanelet.adj_left_same_direction is not None:
        current_lanelet = lanelet_network.find_lanelet_by_id(current_lanelet.adj_left)
        if LaneletType.MAIN_CARRIAGE_WAY in current_lanelet.lanelet_type:
            return current_lanelet.lanelet_id


def convert_to_curvilinear_state(
        position: List[float], velocity: float, acceleration: float, jerk: float, orientation: float,
        config: Configuration) -> Union[Tuple[StateLongitudinal, StateLateral], Tuple[None, None]]:
    """
    Computes initial state of ego vehicle

    :param config:
    :param position: position of vehicle in cartesian coordinates
    :param velocity: velocity of vehicle
    :param acceleration: acceleration of vehicle
    :param jerk: jerk of vehicle
    :param orientation: orientation of vehicle
    :return: lateral and longitudinal state of vehicle
    """
    try:
        s, d = config.planning.CLCS.convert_to_curvilinear_coords(position[0], position[1])
    except ValueError:
        # print(f"<Convert_to_CLCS_state> Out of projection domain: x = {position[0]}, y = {position[1]}")
        return None, None

    theta = config.planning.route.orientation(s)
    if acceleration is not None and jerk is not None:
        x_lon = StateLongitudinal(s=s, v=velocity, a=acceleration, j=jerk)
    elif acceleration is not None:
        x_lon = StateLongitudinal(s=s, v=velocity, a=acceleration)
    else:
        x_lon = StateLongitudinal(s=s, v=velocity)
    x_lat = StateLateral(d=d, theta=(orientation - theta))

    return x_lon, x_lat
