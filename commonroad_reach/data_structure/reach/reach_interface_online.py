import os
import pickle
from collections import defaultdict
from datetime import datetime
from typing import Dict, List

import numpy as np

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


class OnlineReachableSetInterface:
    """Interface to work with reachable sets with python backend."""

    def __init__(self, config: Configuration, name_pickle: str):
        self.config = config
        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        self.dict_time_to_drivable_area: Dict[int, List[ReachPolygon]] = defaultdict(list)
        self.dict_time_to_reachable_set: Dict[int, List[ReachNode]] = defaultdict(list)

        self._restore_reachability_graph(name_pickle)
        self._compute_translation_over_time()

    def _restore_reachability_graph(self, name_pickle):
        dict_time_to_list_tuples_reach_node_attributes, dict_time_to_adjacency_matrices = \
            self.load_offline_computation_result(name_pickle)

        duration_offline = max(dict_time_to_list_tuples_reach_node_attributes)
        assert duration_offline >= self.time_step_end, \
            "<OnlineReachableSetInterface> Scenario duration is longer than offline computation result"

        self._restore_reachable_sets(dict_time_to_list_tuples_reach_node_attributes, dict_time_to_adjacency_matrices)

    def load_offline_computation_result(self, name_pickle):
        print("Loading offline computation result...")
        dict_data = pickle.load(open(os.path.join(self.config.general.path_offline_data, name_pickle), "rb"))

        return dict_data["node_attributes"], dict_data["adjacency_matrices"]

    def _restore_reachable_sets(self, dict_time_to_list_tuples_reach_node_attributes, dict_time_to_adjacency_matrices):
        for time_step, list_tuples_attribute in dict_time_to_list_tuples_reach_node_attributes.items():
            for tuple_attribute in list_tuples_attribute:
                p_lon_min, p_lat_min, p_lon_max, p_lat_max, v_lon_min, v_lat_min, v_lon_max, v_lat_max = tuple_attribute
                polygon_lon = ReachPolygon.from_rectangle_vertices(p_lon_min, v_lon_min, p_lon_max, v_lon_max)
                polygon_lat = ReachPolygon.from_rectangle_vertices(p_lat_min, v_lat_min, p_lat_max, v_lat_max)
                node = ReachNode(polygon_lon, polygon_lat, time_step)

                self.dict_time_to_reachable_set[time_step].append(node)

            if time_step >= 1:
                matrix_adjacency_sparse = dict_time_to_adjacency_matrices[time_step]
                matrix_adjacency_dense = matrix_adjacency_sparse.todense()
                list_nodes_parent = self.dict_time_to_reachable_set[time_step - 1]
                list_nodes_child = self.dict_time_to_reachable_set[time_step]

                for idx_child, list_adjacency in enumerate(matrix_adjacency_dense):
                    node_child = list_nodes_child[idx_child]
                    [list_adjacency] = list_adjacency.tolist()

                    for idx_parent, adjacency in enumerate(list_adjacency):
                        node_parent = list_nodes_parent[idx_parent]
                        if adjacency:
                            node_child.add_parent_node(node_parent)
                            node_parent.add_child_node(node_child)

    def _compute_translation_over_time(self):
        self._dict_time_to_translation = dict()

        p_init = np.array([self.config.planning.p_lon_initial, self.config.planning.p_lat_initial])
        o_init = self.config.planning.orientation_initial
        v_init = np.array([np.cos(o_init), np.sin(o_init)]) * self.config.planning.v_lon_initial
        dt = self.config.planning.dt
        for time_step in range(self.time_step_start, self.time_step_end + 1):
            self._dict_time_to_translation[time_step] = v_init * (time_step * dt) + p_init

    def reachable_set_at_time_step(self, time_step: int):
        assert time_step <= self.time_step_end, "Time step is not valid."
        return self.dict_time_to_reachable_set[time_step]

    def drivable_area_at_time_step(self, time_step: int):
        assert time_step <= self.time_step_end, "Time step is not valid."
        return self.dict_time_to_drivable_area[time_step]

    def compute_reachable_sets(self, time_step_start: int = 1, time_step_end: int = 0):
        """Computes reachable sets for the specified time steps."""
        assert time_step_start != 0, "<OnlineReachableSetInterface> Time step should not start with 0."

        if not time_step_end:
            time_step_end = self.time_step_end

        for time_step in range(time_step_start, time_step_end + 1):
            self._compute_at_time_step(time_step)

        # self._print_analysis()

    def _compute_at_time_step(self, time_step: int):
        self._compute_reachable_set_at_time_step(time_step)

    def _compute_reachable_set_at_time_step(self, time_step):
        """Compute reachable set of the time step."""
        x_translate, y_translate = self._dict_time_to_translation[time_step]

        for node in self.dict_time_to_reachable_set[time_step]:
            p_lon_min, v_lon_min, p_lon_max, v_lon_max = node.polygon_lon.bounds
            p_lat_min, v_lat_min, p_lat_max, v_lat_max = node.polygon_lat.bounds

            p_lon_min_translated = p_lon_min + x_translate
            p_lon_max_translated = p_lon_max + x_translate
            p_lat_min_translated = p_lat_min + y_translate
            p_lat_max_translated = p_lat_max + y_translate

            node.polygon_lon = \
                ReachPolygon.from_rectangle_vertices(p_lon_min_translated, v_lon_min,
                                                     p_lon_max_translated, v_lon_max)
            node.polygon_lat = \
                ReachPolygon.from_rectangle_vertices(p_lat_min_translated, v_lat_min,
                                                     p_lat_max_translated, v_lat_max)

            # self.dict_time_to_drivable_area[time_step].append(node.position_rectangle)
            self.dict_time_to_drivable_area[time_step].append(
                ReachPolygon.from_rectangle_vertices(p_lon_min_translated, p_lat_min_translated,
                                                     p_lon_max_translated, p_lat_max_translated))

    #
    # def _print_analysis(self):
    #     sum_nodes = 0
    #     for time_step in self.dict_time_to_reachable_set:
    #         sum_nodes += len(self.reachable_set_at_time_step(time_step))
    #
    #     print(f"#nodes: {sum_nodes}")
