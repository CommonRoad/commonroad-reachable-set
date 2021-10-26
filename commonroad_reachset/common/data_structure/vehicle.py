# taken from CommonRoad Traffic Rule Monitor

import enum
from typing import Union, Set, Dict, List

import crmonitor_cpp
import numpy as np
from commonroad.geometry.shape import Shape, Rectangle
from commonroad.scenario.obstacle import ObstacleType, SignalState
from commonroad.scenario.trajectory import State
from crmonitor.common.road_network import Lane


class StateLongitudinal:
    """
    Longitudinal state in curvilinear coordinate system
    """
    __slots__ = ['s', 'v', 'a', 'j']

    def __init__(self, **kwargs):
        """ Elements of state vector are determined during runtime."""
        for (field, value) in kwargs.items():
            setattr(self, field, value)

    @property
    def attributes(self) -> List[str]:
        """ Returns all dynamically set attributes of an instance of State.

        :return: subset of slots which are dynamically assigned to the object.
        """
        attributes = list()
        for slot in self.__slots__:
            if hasattr(self, slot):
                attributes.append(slot)
        return attributes

    def __repr__(self):
        state = ''
        for attr in self.attributes:
            state += attr
            state += '= {} '.format(self.__getattribute__(attr))
        return state


class StateLateral:
    """
    Lateral state in curvilinear coordinate system
    """
    __slots__ = ['d', 'theta', 'kappa', 'kappa_dot']

    def __init__(self, **kwargs):
        """ Elements of state vector are determined during runtime."""
        for (field, value) in kwargs.items():
            setattr(self, field, value)

    @property
    def attributes(self) -> List[str]:
        """ Returns all dynamically set attributes of an instance of State.

        :return: subset of slots which are dynamically assigned to the object.
        """
        attributes = list()
        for slot in self.__slots__:
            if hasattr(self, slot):
                attributes.append(slot)
        return attributes

    def __repr__(self):
        state = '\n'
        for attr in self.attributes:
            state += attr
            state += '= {}\n'.format(self.__getattribute__(attr))
        return state


class Input:
    """
    Lateral and longitudinal vehicle input
    """
    __slots__ = ['a', 'kappa_dot_dot']

    @property
    def attributes(self) -> List[str]:
        """ Returns all dynamically set attributes of an instance of State.

        :return: subset of slots which are dynamically assigned to the object.
        """
        attributes = list()
        for slot in self.__slots__:
            if hasattr(self, slot):
                attributes.append(slot)
        return attributes

    def __str__(self):
        state = '\n'
        for attr in self.attributes:
            state += attr
            state += '= {}\n'.format(self.__getattribute__(attr))
        return state


@enum.unique
class VehicleClassification(enum.Enum):
    EGO_VEHICLE = 0
    CROSSING_VEHICLE = 1
    ADJACENT_VEHICLE = 2


class Vehicle:
    """
    Representation of a vehicle with state and input profiles and other information for complete simulation horizon
    """

    def __init__(self, states_lon: Dict[int, StateLongitudinal],
                 states_lat: Dict[int, StateLateral],
                 shape: Union[Shape, Rectangle],
                 cr_states: Dict[int, State],
                 vehicle_id: int,
                 obstacle_type: ObstacleType,
                 lanelet_assignments: Dict[int, Set[int]],
                 signal_states: Dict[int, SignalState] = None,
                 vehicle_classification: Dict[int, VehicleClassification] = None,
                 lane: Union[Lane, List[Lane]] = None,
                 dict_time_to_spot_prediction_occupancy: Dict = None,
                 dict_time_to_spot_extrema: Dict = None,
                 use_spot: bool = False):
        """
        :param states_lon: list of longitudinal states for initialization
        :param states_lat: list of lateral states for initialization
        :param shape: CommonRoad shape of vehicle
        :param cr_states: initial CommonRoad state of vehicle
        :param vehicle_id: id of vehicle
        :param obstacle_type: type of the vehicle, e.g. parked car, car, bus, ...
        :param lanelet_assignments: initial lanelet assignment
        :param signal_states: initial signal state of vehicle
        """
        self._states_lon = states_lon
        self._states_lat = states_lat
        self._states_cr = cr_states
        self._lanelet_assignment = lanelet_assignments
        self._signal_series = signal_states
        self._shape = shape
        self._id = vehicle_id
        self._obstacle_type = obstacle_type
        self._vehicle_classification = vehicle_classification
        self._lane = lane
        self._dict_time_to_spot_prediction_occupancy = dict_time_to_spot_prediction_occupancy
        self._dict_time_to_spot_extrema = dict_time_to_spot_extrema
        self._use_spot = use_spot

    def __repr__(self):
        return f"Vehicle(id={self.id})"

    @property
    def shape(self) -> Rectangle:
        return self._shape

    @property
    def id(self) -> int:
        return self._id

    @property
    def states_lon(self) -> Dict[int, StateLongitudinal]:
        return self._states_lon

    @property
    def states_lat(self) -> Dict[int, StateLateral]:
        return self._states_lat

    @property
    def states_cr(self) -> Dict[int, State]:
        return self._states_cr

    @property
    def state_list_cr(self) -> List[State]:
        state_list = []
        for state in self._states_cr.values():
            state_list.append(state)
        return state_list

    @property
    def obstacle_type(self) -> ObstacleType:
        return self._obstacle_type

    @property
    def lanelet_assignment(self) -> Dict[int, Set[int]]:
        return self._lanelet_assignment

    @property
    def signal_series(self) -> Dict[int, SignalState]:
        return self._signal_series

    @property
    def lane(self) -> Lane:
        return self._lane

    def spot_prediction_occupancy_at_time(self, time_step: int):
        return self._dict_time_to_spot_prediction_occupancy[time_step]

    @lane.setter
    def lane(self, lane: Lane):
        self._lane = lane

    def rear_s(self, time_step: int) -> float:
        """
        Calculates rear s-coordinate of vehicle

        :param time_step: time step to consider
        :returns rear s-coordinate [m]
        """
        s = self._states_lon[time_step].s
        d = self.states_lat[time_step].d
        w = self.shape.width
        l = self.shape.length
        theta = self.states_lat[time_step].theta

        return crmonitor_cpp.rear_s(d, l, s, theta, w)

    @staticmethod
    def calc_rear_s(length, s, theta, width):
        """
        Calculates rear s-coordinate of vehicle

        :param length: length of vehicle
        :param s: longitudinal position
        :param theta: orientation of vehicle
        :param width: width of vehicle
        :returns rear s-coordinate [m]
        """
        return min((length / 2) * np.cos(theta) - (width / 2) * np.sin(theta) + s,
                   (length / 2) * np.cos(theta) - (-width / 2) * np.sin(theta) + s,
                   (-length / 2) * np.cos(theta) - (width / 2) * np.sin(theta) + s,
                   (-length / 2) * np.cos(theta) - (-width / 2) * np.sin(theta) + s)

    def front_s(self, time_step: int) -> float:
        """
        Calculates front s-coordinate of vehicle

        :param time_step: time step to consider
        :returns front s-coordinate [m]
        """
        s = self._states_lon[time_step].s
        d = self.states_lat[time_step].d
        w = self.shape.width
        l = self.shape.length
        theta = self.states_lat[time_step].theta

        return crmonitor_cpp.front_s(d, l, s, theta, w)

    @staticmethod
    def calc_front_s(length, s, theta, width):
        """
        Calculates front s-coordinate of vehicle

        :param length: length of vehicle
        :param s: longitudinal position
        :param theta: orientation of vehicle
        :param width: width of vehicle
        :returns front s-coordinate [m]
        """
        return max((length / 2) * np.cos(theta) - (width / 2) * np.sin(theta) + s,
                   (length / 2) * np.cos(theta) - (-width / 2) * np.sin(theta) + s,
                   (-length / 2) * np.cos(theta) - (width / 2) * np.sin(theta) + s,
                   (-length / 2) * np.cos(theta) - (-width / 2) * np.sin(theta) + s)

    def right_d(self, time_step: int) -> float:
        """
        Calculates right d-coordinate of vehicle

        :param time_step: time step to consider
        :returns right d-coordinate [m]
        """
        d = self.states_lat[time_step].d
        width = self.shape.width
        length = self.shape.length
        theta = self.states_lat[time_step].theta
        return min((width / 2) * np.cos(theta) - (length / 2) * np.sin(theta) + d,
                   (width / 2) * np.cos(theta) - (-length / 2) * np.sin(theta) + d,
                   (-width / 2) * np.cos(theta) - (length / 2) * np.sin(theta) + d,
                   (-width / 2) * np.cos(theta) - (-length / 2) * np.sin(theta) + d)

    def left_d(self, time_step: int) -> float:
        """
        Calculates left d-coordinate of vehicle

        :param time_step: time step to consider
        :returns left d-coordinate [m]
        """
        d = self.states_lat[time_step].d
        width = self.shape.width
        length = self.shape.length
        theta = self.states_lat[time_step].theta
        return max((width / 2) * np.cos(theta) - (length / 2) * np.sin(theta) + d,
                   (width / 2) * np.cos(theta) - (-length / 2) * np.sin(theta) + d,
                   (-width / 2) * np.cos(theta) - (length / 2) * np.sin(theta) + d,
                   (-width / 2) * np.cos(theta) - (-length / 2) * np.sin(theta) + d)

    def append_time_step(self, time_step: int, state_lon: StateLongitudinal, state_lat: StateLateral, state_cr: State,
                         lanelet_assignment: Set[int], signal_state: SignalState = None):
        """
        Adds information for a specific time step to vehicle

        :param time_step: time step of new data
        :param state_lon: longitudinal state to append
        :param state_lat: lateral state to append
        :param state_cr: CommonRoad state to append
        :param lanelet_assignment: lanelet assignment to append
        :param signal_state: signal state to append
        """
        self._states_lon[time_step] = state_lon
        self._states_lat[time_step] = state_lat
        self._states_cr[time_step] = state_cr
        self._lanelet_assignment[time_step] = lanelet_assignment
        self._signal_series[time_step] = signal_state

    def p_lon(self, time_step: int):
        return self.states_lon[time_step].s

    def p_lat(self, time_step: int):
        return self.states_lat[time_step].d

    def v_lon(self, time_step: int):
        return self.states_lon[time_step].v

    def p_lon_min(self, time_step: int, half_length_ego: float = 0):
        if self._use_spot:
            p_lon_min = self._dict_time_to_spot_extrema[time_step][0]

        else:
            p_lon_min = self.rear_s(time_step)

        return p_lon_min - half_length_ego

    def p_lon_max(self, time_step: int, half_length_ego: float = 0):
        if self._use_spot:
            p_lon_max = self._dict_time_to_spot_extrema[time_step][2]

        else:
            p_lon_max = self.front_s(time_step)

        return p_lon_max + half_length_ego

    def p_lat_min(self, time_step: int, half_width_ego: float = 0):
        if self._use_spot:
            p_lat_min = self._dict_time_to_spot_extrema[time_step][1]

        else:
            p_lat_min = self.right_d(time_step)

        return p_lat_min - half_width_ego

    def p_lat_max(self, time_step: int, half_width_ego: float = 0):
        if self._use_spot:
            p_lat_max = self._dict_time_to_spot_extrema[time_step][3]

        else:
            p_lat_max = self.left_d(time_step)

        return p_lat_max + half_width_ego
