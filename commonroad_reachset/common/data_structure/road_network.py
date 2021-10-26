from typing import List, Set

import numpy as np
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet, LaneletType
from commonroad_route_planner.route import Route
from commonroad_route_planner.utility.route import chaikins_corner_cutting, resample_polyline
from pycrccosy import CurvilinearCoordinateSystem


class Lane:
    """
    Lane representation build from several lanelets
    """

    def __init__(self, lanelet_merged: Lanelet, list_ids_lanelets_merged: List[int]):
        """
        :param lanelet_merged: the longer lanelet created by connecting successor lanelets
        :param list_ids_lanelets_merged: list of ids of lanelets that have been merged into one lanelet
        """
        self._lanelet = lanelet_merged
        self._contained_lanelets = set(list_ids_lanelets_merged)
        self._clcs = Lane.create_curvilinear_coordinate_system_from_reference(lanelet_merged.center_vertices)
        self._orientation = self._compute_orientation_from_polyline(lanelet_merged.center_vertices)
        self._curvature = self._compute_curvature_from_polyline(lanelet_merged.center_vertices)
        self._path_length = self._compute_path_length_from_polyline(lanelet_merged.center_vertices)
        self._width = self._compute_width_from_lanelet_boundary(lanelet_merged.left_vertices,
                                                                lanelet_merged.right_vertices)

    @classmethod
    def create_from_route(cls, route: Route):
        pass

    @property
    def lanelet(self) -> Lanelet:
        return self._lanelet

    @property
    def contained_lanelets(self) -> Set[int]:
        return self._contained_lanelets

    @property
    def clcs(self) -> CurvilinearCoordinateSystem:
        return self._clcs

    def orientation(self, position) -> float:
        """
        Calculates orientation of lane given a longitudinal position along lane

        :param position: longitudinal position
        :returns orientation of lane at a given position
        """
        return np.interp(position, self._path_length, self._orientation)

    def width(self, s_position: float) -> float:
        """
        Calculates width of lane given a longitudinal position along lane

        :param s_position: longitudinal position
        :returns width of lane at a given position
        """
        return np.interp(s_position, self._path_length, self._width)

    @staticmethod
    def _compute_orientation_from_polyline(polyline: np.ndarray) -> np.ndarray:
        """
        Computes orientation along a polyline

        :param polyline: polyline for which orientation should be calculated
        :return: orientation along polyline
        """
        assert isinstance(polyline, np.ndarray) and len(polyline) > 1 and polyline.ndim == 2 and len(
            polyline[0, :]) == 2, '<Math>: not a valid polyline. polyline = {}'.format(polyline)
        if len(polyline) < 2:
            raise ValueError('Cannot create orientation from polyline of length < 2')

        orientation = [0]
        for i in range(1, len(polyline)):
            pt1 = polyline[i - 1]
            pt2 = polyline[i]
            tmp = pt2 - pt1
            orientation.append(np.arctan2(tmp[1], tmp[0]))

        return np.array(orientation)

    @staticmethod
    def _compute_curvature_from_polyline(polyline: np.ndarray) -> np.ndarray:
        """
        Computes curvature along a polyline

        :param polyline: polyline for which curvature should be calculated
        :return: curvature along  polyline
        """
        assert isinstance(polyline, np.ndarray) and polyline.ndim == 2 and len(
            polyline[:, 0]) > 2, 'Polyline malformed for curvature computation p={}'.format(polyline)

        x_d = np.gradient(polyline[:, 0])
        x_dd = np.gradient(x_d)
        y_d = np.gradient(polyline[:, 1])
        y_dd = np.gradient(y_d)

        return (x_d * y_dd - x_dd * y_d) / ((x_d ** 2 + y_d ** 2) ** (3. / 2.))

    @staticmethod
    def _compute_path_length_from_polyline(polyline: np.ndarray) -> np.ndarray:
        """
        Computes the path length of a polyline

        :param polyline: polyline for which path length should be calculated
        :return: path length along polyline
        """
        assert isinstance(polyline, np.ndarray) and polyline.ndim == 2 and len(
            polyline[:, 0]) > 2, 'Polyline malformed for pathlenth computation p={}'.format(polyline)

        distance = np.zeros((len(polyline),))
        for i in range(1, len(polyline)):
            distance[i] = distance[i - 1] + np.linalg.norm(polyline[i] - polyline[i - 1])

        return np.array(distance)

    @staticmethod
    def _compute_width_from_lanelet_boundary(left_polyline: np.ndarray, right_polyline: np.ndarray) -> np.ndarray:
        """
        Computes the width of a lanelet

        :param left_polyline: left boundary of lanelet
        :param right_polyline: right boundary of lanelet
        :return: width along lanelet
        """
        width_along_lanelet = np.zeros((len(left_polyline),))
        for i in range(len(left_polyline)):
            width_along_lanelet[i] = np.linalg.norm(left_polyline[i] - right_polyline[i])
        return width_along_lanelet

    @staticmethod
    def create_curvilinear_coordinate_system_from_reference(ref_path: np.array) \
            -> CurvilinearCoordinateSystem:
        """
        Generates curvilinear coordinate system for a reference path

        :param ref_path: reference path (polyline)
        :returns curvilinear coordinate system for reference path
        """
        ref_path_new = resample_polyline(polyline=ref_path, step=1.0)
        ref_path_new = chaikins_corner_cutting(ref_path_new)
        ref_path_new = resample_polyline(polyline=ref_path_new, step=1.0)

        curvilinear_cosy = CurvilinearCoordinateSystem(ref_path_new)
        return curvilinear_cosy


class RoadNetwork:
    """
    Representation of the complete road network of a CommonRoad scenario abstracted to lanes
    """

    def __init__(self, lanelet_network: LaneletNetwork, length_lane_merge: int):
        """
        :param lanelet_network: CommonRoad lanelet network
        """
        self.lanelet_network = lanelet_network
        self.lanes = self._create_lanes(length_lane_merge)

    def _create_lanes(self, length_lane_merge: int) -> List[Lane]:
        """
        Creates lanes for road network

        """
        list_lanes = []
        list_lanelets_in_lane = []
        set_lanelet_start = set()
        # obtain all lanelets with no predecessor or with a predecessor but has a different lanelet type
        for lanelet in self.lanelet_network.lanelets:
            if len(lanelet.predecessor) == 0:
                set_lanelet_start.add(lanelet)
            else:
                predecessors = [self.lanelet_network.find_lanelet_by_id(pred_id) for pred_id in lanelet.predecessor]
                for pred in predecessors:
                    if not lanelet.lanelet_type == pred.lanelet_type:
                        set_lanelet_start.add(lanelet)

        # create all possible merged lanes starting from each lanelet in the start set
        for lanelet in set_lanelet_start:
            if LaneletType.ACCESS_RAMP in lanelet.lanelet_type:
                lanelet_type = LaneletType.ACCESS_RAMP
            elif LaneletType.EXIT_RAMP in lanelet.lanelet_type:
                lanelet_type = LaneletType.EXIT_RAMP
            elif LaneletType.MAIN_CARRIAGE_WAY in lanelet.lanelet_type:
                lanelet_type = LaneletType.MAIN_CARRIAGE_WAY
            else:
                lanelet_type = None
            # compute all lanelets merged by connecting the successors
            list_lanelets_merged, list_ids_lanelets_merged = \
                Lanelet.all_lanelets_by_merging_successors_from_lanelet(lanelet, self.lanelet_network,
                                                                        length_lane_merge)
            # if there is no successor to be merged, add the start lanelet itself
            if len(list_lanelets_merged) == 0 or len(list_ids_lanelets_merged) == 0:
                list_lanelets_merged.append(lanelet)
                list_ids_lanelets_merged.append([lanelet.lanelet_id])

            # add as tuples to list to be used later
            for idx in range(len(list_lanelets_merged)):
                list_lanelets_in_lane.append((list_lanelets_merged[idx], list_ids_lanelets_merged[idx]))

        for lane_element in list_lanelets_in_lane:
            list_lanes.append(Lane(lane_element[0], lane_element[1]))

        return list_lanes

    def find_lane_ids_by_obstacle(self, obstacle_id: int, time_step: int) -> Set[int]:
        """
        Finds the lanes an obstacle belongs to and returns their IDs

        :param obstacle_id: ID of the obstacle
        :param time_step: time step of interest
        """
        lane_ids = set()
        for lane in self.lanes:
            if obstacle_id in lane.lanelet.dynamic_obstacle_by_time_step(time_step):
                lane_ids.add(lane.lanelet.lanelet_id)

        return lane_ids

    def find_lane_ids_by_lanelets(self, lanelets: Set[int]) -> Set[int]:
        """
        Finds the lanes given set of lanelets belong to and returns their IDs

        :param lanelets: list of lanelet IDs
        :returns set of lanelet IDs
        """
        lane_ids = set()
        for lane in self.lanes:
            for lanelet_id in lanelets:
                if lanelet_id in lane.contained_lanelets:
                    lane_ids.add(lane.lanelet.lanelet_id)

        return lane_ids

    def find_lanes_by_lanelets(self, lanelets: Set[int]) -> Set[Lane]:
        """
        Finds the lanes to which a given set of lanelets belongs to

        :param lanelets: list of lanelet IDs
        :returns set of lane objects
        """
        lanes = set()
        for lane in self.lanes:
            for lanelet_id in lanelets:
                if lanelet_id in lane.contained_lanelets:
                    lanes.add(lane)

        return lanes

    def find_lane_by_lanelet(self, lanelet_id: int) -> Lane:
        """
        Finds the lane a lanelet belongs to

        :param lanelet_id: CommonRoad lanelet ID
        :returns lane object
        """
        for lane in self.lanes:
            if lanelet_id in lane.contained_lanelets:
                return lane

    def find_lane_by_obstacle(self, obs_lanelet_center: List[int], obs_lanelet_shape: List[int]) -> Lane:
        """
        Finds the lanes an obstacle occupies

        :param obs_lanelet_center: IDs of lanelet the obstacle center is on (use only first one)
        :param obs_lanelet_shape: IDs of lanelet the obstacle shape is on
        :returns lane the obstacle center is on
        """

        occupied_lanes = set()
        lanelets_center_updated = obs_lanelet_center
        obs_lanelet_shape_updated = obs_lanelet_shape
        if len(obs_lanelet_center) > 0:
            for lane in self.lanes:
                for lanelet in lanelets_center_updated:
                    if lanelet in lane.contained_lanelets:
                        occupied_lanes.add(lane)
        else:
            for lane in self.lanes:
                for lanelet in obs_lanelet_shape_updated:
                    if lanelet in lane.contained_lanelets:
                        occupied_lanes.add(lane)
        if len(occupied_lanes) == 1:
            return list(occupied_lanes)[0]
        for lane in occupied_lanes:
            for lanelet_id in lane.contained_lanelets:
                if LaneletType.MAIN_CARRIAGE_WAY in self.lanelet_network.find_lanelet_by_id(lanelet_id).lanelet_type:
                    return lane
        return list(occupied_lanes)[0]
