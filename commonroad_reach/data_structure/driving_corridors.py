from typing import Union, List, Dict
from collections import defaultdict
import warnings
import math
import time

import numpy as np
import networkx as nx

import pycrreachset
from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.utility.geometry import area_of_reachable_set

# TODO Adapt all data types to work with the Python interface -> Goal: Driving corridor extraction works with both
#  backends
# TODO add correct description of DIGITS
# precision
DIGITS = 2


class DrivingCorridors:
    """
    Class to compute driving corridors based on previous computation of reachable sets and drivable area
    """
    def __init__(self, reachable_sets: Dict[int, List[pycrreachset.ReachNode]], config: Configuration):
        self.config = config
        self.reach_set = reachable_sets
        if config.reachable_set.mode == 3:
            self.backend = "CPP"        # using C++ backend
        else:
            self.backend = "PYTHON"     # using Python backend

    @property
    def reach_set(self):
        return self._reach_set

    @reach_set.setter
    def reach_set(self, reachable_sets):
        # update reach_set and time_steps with new reachable set
        self._reach_set = reachable_sets
        self.time_steps = sorted(list(reachable_sets.keys()))

    def compute_driving_corridor(self,
                                 lon_positions: Union[List[float], None] = None,
                                 lon_driving_corridor: Union[Dict[int, List[pycrreachset.ReachNode]], None] = None)\
            -> List[Dict[int, List[pycrreachset.ReachNode]]]:
        """
        Function identifies driving corridors within the reachable sets. If no parameters are passed, then a
        longitudinal driving corridor is computed. If the optional parameters are passed, a lateral driving corridor is
        computed for a given longitudinal driving corridor.
        :param lon_positions: (optional) longitudinal position
        :param lon_driving_corridor: (optional) longitudinal driving corridor
        """
        # TODO add specific terminal set (e.g., goal region)

        if lon_positions is None and lon_driving_corridor is None:
            print("Computing longitudinal driving corridor...")
            time_start = time.time()
            # compute longitudinal driving corridor
            lon_positions_dict = None
            # determine connected components in last time step
            connected_components = self._determine_connected_components(list(self.reach_set[self.time_steps[-1]]))
        elif lon_positions is not None and lon_driving_corridor is not None:
            print("Computing lateral driving corridor...")
            time_start = time.time()
            # compute lateral driving corridor for given longitudinal driving corridor
            assert (len(lon_positions) == len(self.time_steps))
            lon_positions_dict = dict(zip(self.time_steps, lon_positions))
            # determine reachable sets which contain the longitudinal position in last time step
            overlapping_nodes = self._determine_reachset_overlap_with_longitudinal_position(
                lon_driving_corridor[self.time_steps[-1]], lon_positions_dict[self.time_steps[-1]])
            # then determine connected components within the overlapping reachable sets
            connected_components = self._determine_connected_components(list(overlapping_nodes))
        else:
            err_msg = "Please provide both longitudinal positions and a longitudinal driving corridor if you wish to " \
                      "compute a lateral driving corridor"
            raise ValueError(err_msg)

        # initialize list for driving corridors
        driving_corridor_list = list()
        # initialize list for heuristic
        heuristic = list()

        # create longitudinal driving corridor for each connected set
        for cc in connected_components:
            driving_corridor_node_ids = list()
            graph = nx.DiGraph()
            graph.add_node(1, time_idx=self.time_steps[-1], reach_set=cc)
            self._create_reachset_connected_components_graph_backwards(
                driving_corridor_node_ids, graph, 1, self.time_steps[-1], cc,
                lon_positions_dict, lon_driving_corridor)

            for dc in driving_corridor_node_ids:
                dc_dict = defaultdict(list)
                [dc_dict[graph.nodes[node_id]['time_idx']].extend(graph.nodes[node_id]['reach_set']) for node_id in dc]
                driving_corridor_list.append(dc_dict)
                heuristic.append(self._determine_area_of_driving_corridor(dc_dict))

        if not driving_corridor_list:
            warnings.warn('\t\t\t No driving corridor found!')
        else:
            driving_corridor_list = [elem for _, elem in sorted(zip(heuristic, driving_corridor_list), key=lambda
                pair: pair[0], reverse=True)]

        if self.config.debug.measure_time:
            print(f"\tComputation took: \t{time.time() - time_start:.3f}s")

        return driving_corridor_list

    def _create_reachset_connected_components_graph_backwards(self, driving_corridors_list: List[int], graph: nx.Graph,
                                                              node_id: int, time_idx: int,
                                                              reach_set_nodes: List[pycrreachset.ReachNode],
                                                              lon_pos: Union[Dict[int, float], None] = None,
                                                              lon_driving_corridor: Union[Dict[int, List[pycrreachset.ReachNode]], None] = None):
        """
        Traverses graph of connected reachable sets backwards in time and extracts paths starting from a terminal set.
        A path within the graph corresponds to a possible driving corridor
        :param driving_corridors_list: list of found driving corridors in the reachable set
        :param graph: graph of possible driving corridors
        :param node_id: ID of connected component of the previous time step
        :param time_idx: current time step
        :param reach_set_nodes: reachable set
        :param lon_pos: longitudinal positions of longitudinal trajectory (only necessary for lateral driving corridors)
        :param lon_driving_corridor: longitudinal driving corridor (only necessary for lateral driving corridors)
        """
        longitudinal = False
        lateral = False
        if lon_pos is None and lon_driving_corridor is None:
            longitudinal = True     # use algorithm for longitudinal corridor
        elif lon_pos is not None and lon_driving_corridor is not None:
            lateral = True          # use algorithm for lateral corridor
        else:
            err_msg = "You need to provide both longitudinal positions and a longitudinal driving corridor to " \
                  "compute a lateral driving corridor"
            raise ValueError(err_msg)

        # Terminate if more than 10 driving corridors found
        if len(driving_corridors_list) > 10:
            return

        # if initial time step reached: add initial reachable set node (id=1)
        if time_idx == self.time_steps[0]:
            # nx simple paths: graph, source node id, target node id
            driving_corridors_list.extend(nx.all_simple_paths(graph, node_id, 1))
            return

        # search lateral driving corridors
        if lateral:
            parent_reach_set_nodes = set()
            # determine parent reach set nodes for each reach set node within connected components
            [parent_reach_set_nodes.update(reach_node.vec_nodes_parent()) for reach_node in reach_set_nodes]
            # for lateral driving corridor: further consider only sets that overlap with given longitudinal position
            filtered_parent_reach_set_nodes = self._determine_reachset_overlap_with_longitudinal_position(
                list(parent_reach_set_nodes), lon_pos[time_idx - 1])
            if not filtered_parent_reach_set_nodes:
                warnings.warn('No reachboxes found at x position. # of parent reach nodes: {}. '
                              'current time step {}, max time step {}'.format(
                    len(parent_reach_set_nodes), time_idx, self.time_steps[-1]))

            # filter out reach set nodes that are not part of the longitudinal driving corridor
            filtered_parent_reach_set_nodes.intersection_update(lon_driving_corridor[time_idx - 1])
        # search longitudinal driving corridors
        elif longitudinal:
            parent_reach_set_nodes = set()
            # determine parent reach set nodes for each reach set node within connected components
            [parent_reach_set_nodes.update(reach_node.vec_nodes_parent()) for reach_node in reach_set_nodes]
            filtered_parent_reach_set_nodes = parent_reach_set_nodes
        else:
            filtered_parent_reach_set_nodes = None
            return

        # determine connected components in parent reach sets
        connected_components = self._determine_connected_components(list(filtered_parent_reach_set_nodes))

        # recursion backwards in time
        for cc in connected_components:
            child_id = len(graph) + 1
            graph.add_node(child_id, time_idx=time_idx - 1, reach_set=cc)
            graph.add_edge(child_id, node_id)
            self._create_reachset_connected_components_graph_backwards(
                driving_corridors_list,
                graph,
                child_id,
                time_idx - 1,
                cc,
                lon_pos,
                lon_driving_corridor)

    def _determine_connected_components(self, reach_set_nodes: List[pycrreachset.ReachNode]):
        """
        Determines and returns the connected reachable sets in positions domain within the list of given base sets (i.e.,
        reachable set nodes)
        :param reach_set_nodes: list of reachable set nodes
        :return: list of connected reachable sets
        """
        # determine overlapping reachable set nodes
        # TODO implement python function for connected set identification (using shapely polygon)
        if self.backend == "CPP":
            overlap = pycrreachset.connected_reachset_boost(reach_set_nodes, DIGITS)
        elif self.backend == "PYTHON":
            overlap = None
        else:
            err_msg = "Invalid backend for driving corridor extraction. One of the following is supported: CPP or PYTHON"
            raise ValueError(err_msg)
        adjacency = []
        [adjacency.extend(v) for v in overlap.values()]

        # create graph with nodes = reach set nodes and edges = adjacency
        graph = nx.Graph()
        graph.add_nodes_from(list(range(0, len(reach_set_nodes))))
        graph.add_edges_from(adjacency)
        # retrieve connected components and sort according to heuristic (here area of connected reachable sets)
        connected_reach_sets = list()
        heuristic = list()
        for i, connected_reach_set_nodes_idx in enumerate(nx.connected_components(graph)):
            connected_reach_sets.append([reach_set_nodes[j] for j in connected_reach_set_nodes_idx])
            heuristic.append(area_of_reachable_set(connected_reach_sets[-1]))

        connected_reach_sets_list = [elem for _, elem in sorted(zip(heuristic, connected_reach_sets),
                                                                key=lambda pair: pair[0], reverse=True)]
        return connected_reach_sets_list

    @staticmethod
    def _determine_reachset_overlap_with_longitudinal_position(reach_set_nodes: List[pycrreachset.ReachNode],
                                                               lon_pos: float):
        """
        Checks which drivable areas of the given reachable sets contain a given longitudinal position and returns the
        corresponding reachable sets
        :param reach_set_nodes: List of reachable set nodes
        :param lon_pos: given longitudinal positions
        :return reach_set_nodes_overlap: Set containing the reachable set nodes which overlap with longitudinal position
        """
        reach_set_nodes_overlap = set()
        for reach_node in reach_set_nodes:
            if np.greater_equal(round(lon_pos * 10.0 ** DIGITS), math.floor(reach_node.x_min() * 10.0 ** DIGITS)) and \
                    np.greater_equal(math.ceil(reach_node.x_max() * 10.0 ** DIGITS), round(lon_pos * 10.0 ** DIGITS)):
                reach_set_nodes_overlap.add(reach_node)
        return reach_set_nodes_overlap

    @staticmethod
    def _determine_area_of_driving_corridor(driving_corridor: Dict[int, List[pycrreachset.ReachNode]]):
        """
        Function to compute the cumulative area of a driving corridor, i.e.,
        :param driving_corridor:
        :return: area
        """
        area = 0.0
        for time_idx, reach_set_nodes in driving_corridor.items():
            area += area_of_reachable_set(reach_set_nodes)
        return area

