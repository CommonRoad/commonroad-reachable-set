import logging
import os
import pickle
import time

import numpy as np
from scipy import sparse

logger = logging.getLogger(__name__)
from typing import List

from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon

from collections import defaultdict

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_analysis_offline import OfflineReachabilityAnalysis


class PyGridOfflineReachableSet:
    """Offline step in the multi-step reachable set computation with Python backend."""

    def __init__(self, config: Configuration):
        self.config = config
        if config.planning.coordinate_system != "CART":
            message = "Multi-step reachable set computation only supports Cartesian coordinate system."
            logger.error(message)
            raise Exception(message)

        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        self._reachability_analysis = OfflineReachabilityAnalysis(config)
        self._dict_time_to_base_set_propagated = defaultdict(list)
        self._dict_time_to_drivable_area = defaultdict(list)
        self._dict_time_to_reachable_set = defaultdict(list)
        self._dict_time_to_drivable_area[self.time_step_start] = self._reachability_analysis.initial_drivable_area
        self._dict_time_to_reachable_set[self.time_step_start] = self._reachability_analysis.initial_reachable_set
        self._list_time_steps_computed = [0]

    def drivable_area_at_time_step(self, time_step: int) -> List[ReachPolygon]:
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for drivable area retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self._dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int) -> List[ReachNodeMultiGeneration]:
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for reachable set retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self._dict_time_to_reachable_set[time_step]

    def compute_reachable_sets(self, time_step_start: int, time_step_end: int):
        """Computes reachable sets for the specified time steps."""
        for time_step in range(time_step_start, time_step_end + 1):
            message = f"\tTime step: {time_step}"
            print(message)
            logger.debug(message)

            time_start = time.time()
            self._compute_at_time_step(time_step)
            self._list_time_steps_computed.append(time_step)

            message = f"\t#Nodes: {len(self.reachable_set_at_time_step(time_step))}, " \
                      f"computation took: {time.time() - time_start:.3f}s"
            print(message)
            logger.debug(message)

        self._reachability_analysis.determine_grandparent_relationship(self._dict_time_to_reachable_set)

        # save computation result to pickle file
        self._save_to_pickle()

    def _compute_at_time_step(self, time_step: int):
        """Computes drivable area and reachable set of the time step."""
        self._compute_drivable_area_at_time_step(time_step)
        self._compute_reachable_set_at_time_step(time_step)

    def _compute_drivable_area_at_time_step(self, time_step: int):
        """Computes drivable area of the time step.

        Drivable area is computed based on reachable set of the last time step.
        """
        reachable_set_previous = self._dict_time_to_reachable_set[time_step - 1]
        drivable_area, base_set_propagated = \
            self._reachability_analysis.compute_drivable_area_at_time_step(time_step, reachable_set_previous)

        self._dict_time_to_drivable_area[time_step] = drivable_area
        self._dict_time_to_base_set_propagated[time_step] = base_set_propagated

    def _compute_reachable_set_at_time_step(self, time_step):
        """Compute reachable set of the time step.

        Reachable set is computed based on the drivable area and the list of propagated nodes. It is also based on the
        reachable set of the last time step.
        """
        base_sets_propagated = self._dict_time_to_base_set_propagated[time_step]
        drivable_area = self._dict_time_to_drivable_area[time_step]
        reachable_set = self._reachability_analysis.compute_reachable_set_at_time_step(time_step,
                                                                                       base_sets_propagated,
                                                                                       drivable_area)
        self._dict_time_to_reachable_set[time_step] = reachable_set

    def _save_to_pickle(self):
        """Saves computation result as a pickle file."""
        # todo: save vertices instead of min/max values
        # create output directory
        os.makedirs(self.config.general.path_offline_data, exist_ok=True)

        dict_data = dict()
        dict_time_to_list_tuples_reach_node_attributes, \
        dict_time_to_adjacency_matrices_parent, \
        dict_time_to_adjacency_matrices_grandparent = self._extract_information()

        dict_data["node_attributes"] = dict_time_to_list_tuples_reach_node_attributes
        dict_data["adjacency_matrices_parent"] = dict_time_to_adjacency_matrices_parent
        dict_data["adjacency_matrices_grandparent"] = dict_time_to_adjacency_matrices_grandparent

        time_steps = self.config.planning.time_steps_computation
        size_grid = self.config.reachable_set.size_grid
        a_max = self.config.vehicle.ego.a_max
        v_max = self.config.vehicle.ego.v_max

        name_file = f"offline_{time_steps}_{size_grid}_{a_max}_{v_max}.pickle"
        f = open(f"{self.config.general.path_offline_data}{name_file}", 'wb')
        pickle.dump(dict_data, f)
        f.close()

        print()
        message = f"Computation result saved to pickle file: {name_file}"
        print(message)
        logger.info(message)

    def _extract_information(self):
        """Extracts essential information from the computation result."""
        dict_time_to_list_tuples_reach_node_attributes = defaultdict(list)
        dict_time_to_adjacency_matrices_parent = dict()
        dict_time_to_adjacency_matrices_grandparent = dict()

        for time_step, list_nodes in self._dict_time_to_reachable_set.items():
            for node in list_nodes:
                tuple_attribute = (
                    round(node.p_lon_min, 6), round(node.p_lat_min, 6), round(node.p_lon_max, 6),
                    round(node.p_lat_max, 6),
                    round(node.v_lon_min, 6), round(node.v_lat_min, 6), round(node.v_lon_max, 6),
                    round(node.v_lat_max, 6))
                dict_time_to_list_tuples_reach_node_attributes[time_step].append(tuple_attribute)

            # parent-child relationship
            if time_step >= 1:
                list_nodes_parent = self._dict_time_to_reachable_set[time_step - 1]

                matrix_adjacency = list()
                for node in list_nodes:
                    list_adjacency = [(node_parent in node.list_nodes_parent) for node_parent in list_nodes_parent]
                    matrix_adjacency.append(list_adjacency)

                matrix_adjacency_dense = np.array(matrix_adjacency)
                matrix_adjacency_sparse = sparse.csr_matrix(matrix_adjacency_dense, dtype=bool)
                dict_time_to_adjacency_matrices_parent[time_step] = matrix_adjacency_sparse

            # grandparent-grandchild relationship
            if time_step >= 2:
                list_nodes_grandparent = self._dict_time_to_reachable_set[time_step - 2]

                matrix_adjacency = list()
                for node in list_nodes:
                    list_adjacency = [(node_grandparent in node.list_nodes_grandparent)
                                      for node_grandparent in list_nodes_grandparent]
                    matrix_adjacency.append(list_adjacency)

                matrix_adjacency_dense = np.array(matrix_adjacency)
                matrix_adjacency_sparse = sparse.csr_matrix(matrix_adjacency_dense, dtype=bool)
                dict_time_to_adjacency_matrices_grandparent[time_step] = matrix_adjacency_sparse

        return dict_time_to_list_tuples_reach_node_attributes, \
               dict_time_to_adjacency_matrices_parent, \
               dict_time_to_adjacency_matrices_grandparent
