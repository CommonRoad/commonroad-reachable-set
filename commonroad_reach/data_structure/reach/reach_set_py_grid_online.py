import os
import pickle
from collections import defaultdict

import numpy as np

from commonroad_reach.data_structure.collision_checker_py import PyCollisionChecker
from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


class PyGridOnlineReachableSet:
    """Interface to work with reachable sets with python backend."""

    def __init__(self, config: Configuration):
        self.config = config
        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        self._dict_time_to_drivable_area = defaultdict(list)
        self._dict_time_to_reachable_set = defaultdict(list)
        self._dict_time_to_drivable_area_pruned = defaultdict(list)
        self._dict_time_to_reachable_set_pruned = defaultdict(list)
        self._prune_reachable_set = config.reachable_set.prune_nodes_not_reaching_final_time_step
        self._list_time_steps_computed = [0]

        self._restore_reachability_graph()
        self._compute_translation_over_time()
        self._initialize_collision_checker()

    def drivable_area_at_time_step(self, time_step: int):
        if time_step not in self._list_time_steps_computed:
            print("Given time step for drivable area retrieval is out of range.")
            return []

        else:
            if self._prune_reachable_set:
                return self._dict_time_to_drivable_area_pruned[time_step]

            else:
                return self._dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int):
        if time_step not in self._list_time_steps_computed:
            print("Given time step for reachable set retrieval is out of range.")
            return []

        else:
            if self._prune_reachable_set:
                return self._dict_time_to_reachable_set_pruned[time_step]

            else:
                return self._dict_time_to_reachable_set[time_step]

    def _restore_reachability_graph(self):
        dict_time_to_list_tuples_reach_node_attributes, dict_time_to_adjacency_matrices_parent, \
        dict_time_to_adjacency_matrices_grandparent = self.load_offline_computation_result()

        self._restore_reachable_sets(dict_time_to_list_tuples_reach_node_attributes,
                                     dict_time_to_adjacency_matrices_parent,
                                     dict_time_to_adjacency_matrices_grandparent)

    def load_offline_computation_result(self):
        print("Loading offline computation result...")
        path_file_pickle = os.path.join(self.config.general.path_offline_data,
                                        self.config.reachable_set.name_pickle_offline)
        dict_data = pickle.load(open(path_file_pickle, "rb"))

        return dict_data["node_attributes"], dict_data["adjacency_matrices_parent"], \
               dict_data["adjacency_matrices_grandparent"]

    def _restore_reachable_sets(self, dict_time_to_list_tuples_reach_node_attributes,
                                dict_time_to_adjacency_matrices_parent,
                                dict_time_to_adjacency_matrices_grandparent):
        for time_step, list_tuples_attribute in dict_time_to_list_tuples_reach_node_attributes.items():
            for tuple_attribute in list_tuples_attribute:
                # todo: change to vertices of polytopes
                p_x_min, p_y_min, p_x_max, p_y_max, v_x_min, v_y_min, v_x_max, v_y_max = tuple_attribute
                polygon_x = ReachPolygon.from_rectangle_vertices(p_x_min, v_x_min, p_x_max, v_x_max)
                polygon_y = ReachPolygon.from_rectangle_vertices(p_y_min, v_y_min, p_y_max, v_y_max)
                node = ReachNodeMultiGeneration(polygon_x, polygon_y, time_step)

                self._dict_time_to_reachable_set[time_step].append(node)

            if time_step >= 1:
                matrix_adjacency_dense = dict_time_to_adjacency_matrices_parent[time_step].todense()
                list_nodes_parent = self._dict_time_to_reachable_set[time_step - 1]
                list_nodes = self._dict_time_to_reachable_set[time_step]

                for idx_node, list_adjacency in enumerate(matrix_adjacency_dense):
                    node = list_nodes[idx_node]
                    [list_adjacency] = list_adjacency.tolist()

                    for idx_parent, adjacency in enumerate(list_adjacency):
                        node_parent = list_nodes_parent[idx_parent]
                        if adjacency:
                            node.add_parent_node(node_parent)
                            node_parent.add_child_node(node)

            if time_step >= 2:
                matrix_adjacency_dense = dict_time_to_adjacency_matrices_grandparent[time_step].todense()
                list_nodes_grandparent = self._dict_time_to_reachable_set[time_step - 2]
                list_nodes = self._dict_time_to_reachable_set[time_step]

                for idx_node, list_adjacency in enumerate(matrix_adjacency_dense):
                    node = list_nodes[idx_node]
                    [list_adjacency] = list_adjacency.tolist()

                    for idx_grandparent, adjacency in enumerate(list_adjacency):
                        node_grandparent = list_nodes_grandparent[idx_grandparent]
                        if adjacency:
                            node.add_grandparent_node(node_grandparent)
                            node_grandparent.add_grandchild_node(node)

    def _compute_translation_over_time(self):
        self._dict_time_to_translation = dict()

        p_init = np.array([self.config.planning.p_lon_initial, self.config.planning.p_lat_initial])
        o_init = self.config.planning.orientation_initial
        v_init = np.array([np.cos(o_init), np.sin(o_init)]) * self.config.planning.v_lon_initial
        dt = self.config.planning.dt

        for time_step in range(self.time_step_start, self.time_step_end + 1):
            self._dict_time_to_translation[time_step] = v_init * (time_step * dt) + p_init

    def _initialize_collision_checker(self):
        # Python collision checker
        if self.config.reachable_set.mode == 4:
            self._collision_checker = PyCollisionChecker(self.config)

        # C++ collision checker
        elif self.config.reachable_set.mode == 5:
            try:
                from commonroad_reach.data_structure.collision_checker_cpp import CppCollisionChecker

            except ImportError:
                print("Importing C++ collision checker failed.")

            else:
                self._collision_checker = CppCollisionChecker(self.config)

    def compute_reachable_sets(self, time_step_start: int = 1, time_step_end: int = 0):
        """Computes reachable sets for the specified time steps."""
        assert time_step_start != 0, "Time step should not start with 0."

        # duration_offline = max(dict_time_to_list_tuples_reach_node_attributes)
        # assert duration_offline >= self.time_step_end, "Scenario duration is longer than offline computation result"

        if not time_step_end:
            time_step_end = self.time_step_end

        for time_step in range(time_step_start, time_step_end + 1):
            self._compute_at_time_step(time_step)

        # self._print_analysis()

    def _compute_at_time_step(self, time_step: int):
        """Compute reachable set of the time step."""
        self._translate_reachable_set(time_step)
        self._discard_invalid_reachable_set(time_step)
        self._update_drivable_area(time_step)

    def _translate_reachable_set(self, time_step: int):
        x_translate, y_translate = self._dict_time_to_translation[time_step]

        for node in self._dict_time_to_reachable_set[time_step]:
            p_x_min, v_x_min, p_x_max, v_x_max = node.polygon_lon.bounds
            p_y_min, v_y_min, p_y_max, v_y_max = node.polygon_lat.bounds

            p_x_min_translated = p_x_min + x_translate
            p_x_max_translated = p_x_max + x_translate
            p_y_min_translated = p_y_min + y_translate
            p_y_max_translated = p_y_max + y_translate

            node.polygon_lon = ReachPolygon.from_rectangle_vertices(p_x_min_translated, v_x_min,
                                                                    p_x_max_translated, v_x_max)
            node.polygon_lat = ReachPolygon.from_rectangle_vertices(p_y_min_translated, v_y_min,
                                                                    p_y_max_translated, v_y_max)

    def _discard_invalid_reachable_set(self, time_step: int):
        list_idx_nodes_to_be_discarded = []
        for idx, node in enumerate(self._dict_time_to_reachable_set[time_step]):
            if self._collision_checker.collides_at_time_step(time_step, node.position_rectangle):
                list_idx_nodes_to_be_discarded.append(idx)

            if not node.list_nodes_parent and time_step >= 1:
                list_idx_nodes_to_be_discarded.append(idx)

            if not node.list_nodes_grandparent and time_step >= 2:
                list_idx_nodes_to_be_discarded.append(idx)

        if list_idx_nodes_to_be_discarded:
            for idx in list_idx_nodes_to_be_discarded:
                node = self._dict_time_to_reachable_set[time_step][idx]
                for node_parent in node.list_nodes_parent:
                    node_parent.remove_child_node(node)

                for node_child in node.list_nodes_child:
                    node_child.remove_parent_node(node)

                for node_grandparent in node.list_nodes_grandparent:
                    node_grandparent.remove_grandchild_node(node)

                for node_grandchild in node.list_nodes_grandchild:
                    node_grandchild.remove_grandparent_node(node)

            self._dict_time_to_reachable_set[time_step] = \
                [node for idx, node in enumerate(self._dict_time_to_reachable_set[time_step])
                 if idx not in list_idx_nodes_to_be_discarded]

    def _update_drivable_area(self, time_step):
        self._dict_time_to_drivable_area[time_step].clear()
        for node in self.reachable_set_at_time_step(time_step):
            self._dict_time_to_drivable_area[time_step].append(node.position_rectangle)

    def _print_analysis(self):
        sum_nodes = 0
        for time_step in self._dict_time_to_reachable_set:
            sum_nodes += len(self.reachable_set_at_time_step(time_step))

        print(f"#nodes: {sum_nodes}")
