import logging

from commonroad_reach.data_structure.reach.reach_set import ReachableSet

logger = logging.getLogger(__name__)
import os
import pickle
from collections import defaultdict

import numpy as np

from commonroad_reach.data_structure.collision_checker_py import PyCollisionChecker
from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


class PyGridOnlineReachableSet(ReachableSet):
    """Online step in the multi-step reachable set computation with Python backend."""

    def __init__(self, config: Configuration):
        super().__init__(config)
        self.config = config
        if config.planning.coordinate_system != "CART":
            message = "Multi-step reachable set computation only supports Cartesian coordinate system."
            logger.error(message)
            raise Exception(message)

        self._num_time_steps_offline_computation = 0
        self._collision_checker = None

        self._restore_reachability_graph()
        self._compute_translation_over_time()
        self._initialize_collision_checker()

    def _restore_reachability_graph(self):
        """Restores reachability graph from the offline computation result."""
        dict_time_to_list_tuples_reach_node_attributes, dict_time_to_adjacency_matrices_parent, \
        dict_time_to_adjacency_matrices_grandparent = self.load_offline_computation_result()
        self._num_time_steps_offline_computation = len(dict_time_to_list_tuples_reach_node_attributes)

        self._restore_reachable_sets(dict_time_to_list_tuples_reach_node_attributes,
                                     dict_time_to_adjacency_matrices_parent,
                                     dict_time_to_adjacency_matrices_grandparent)

    def load_offline_computation_result(self):
        """Loads pickle file generated in the offline computation step."""
        message = "* Loading offline computation result..."
        print(message)
        logger.info(message)

        path_file_pickle = os.path.join(self.config.general.path_offline_data,
                                        self.config.reachable_set.name_pickle_offline)
        dict_data = pickle.load(open(path_file_pickle, "rb"))

        return dict_data["node_attributes"], dict_data["adjacency_matrices_parent"], \
               dict_data["adjacency_matrices_grandparent"]

    def _restore_reachable_sets(self, dict_time_to_list_tuples_reach_node_attributes,
                                dict_time_to_adjacency_matrices_parent,
                                dict_time_to_adjacency_matrices_grandparent):
        """Restores reachable sets from the offline computation result."""
        # todo: change to vertices of polytopes
        for time_step, list_tuples_attribute in dict_time_to_list_tuples_reach_node_attributes.items():
            # reconstruct nodes in the reachability graph
            for tuple_attribute in list_tuples_attribute:
                p_x_min, p_y_min, p_x_max, p_y_max, v_x_min, v_y_min, v_x_max, v_y_max = tuple_attribute
                polygon_x = ReachPolygon.from_rectangle_vertices(p_x_min, v_x_min, p_x_max, v_x_max)
                polygon_y = ReachPolygon.from_rectangle_vertices(p_y_min, v_y_min, p_y_max, v_y_max)

                node = ReachNodeMultiGeneration(polygon_x, polygon_y, time_step)
                self._dict_time_to_reachable_set[time_step].append(node)

            # restore parent-child relationship
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

            # restore grandparent-grandchild relationship
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
        """Computes translation of the reachable sets at different time steps.

        The translation is based on the initial state of the planning problem.
        """
        self._dict_time_to_translation = defaultdict()

        p_init = np.array([self.config.planning.p_lon_initial, self.config.planning.p_lat_initial])
        o_init = self.config.planning.o_initial
        v_init = np.array([np.cos(o_init), np.sin(o_init)]) * self.config.planning.v_lon_initial
        dt = self.config.planning.dt

        for time_step in range(self.time_step_start, self.time_step_end + 1):
            self._dict_time_to_translation[time_step] = (v_init * (time_step * dt) + p_init, v_init)

    def _initialize_collision_checker(self):
        if self.config.reachable_set.mode == 4:
            self._collision_checker = PyCollisionChecker(self.config)

        elif self.config.reachable_set.mode == 5:
            try:
                from commonroad_reach.data_structure.collision_checker_cpp import CppCollisionChecker

            except ImportError:
                message = "Importing C++ collision checker failed."
                print(message)
                logger.exception(message)

            else:
                self._collision_checker = CppCollisionChecker(self.config)

    def compute_reachable_sets(self, time_step_start: int, time_step_end: int):
        for time_step in range(time_step_start, time_step_end + 1):
            if time_step > self._num_time_steps_offline_computation:
                message = f"Time step {time_step} is out of range, max allowed: {self._num_time_steps_offline_computation}"
                print(message)
                logger.warning(message)
                continue

            self._list_time_steps_computed.append(time_step)
            self._compute_at_time_step(time_step)

        if self.config.reachable_set.prune_nodes_not_reaching_final_time_step:
            self._prune_nodes_not_reaching_final_time_step()

    def _compute_at_time_step(self, time_step: int):
        """Compute reachable set for the given time step."""
        self._translate_reachable_set(time_step)
        self._discard_invalid_reachable_set(time_step)
        self._update_drivable_area(time_step)

    def _translate_reachable_set(self, time_step: int):
        """Translates reachable sets based on the initial state and the time step."""
        p_translate, v_translate = self._dict_time_to_translation[time_step]

        for node in self._dict_time_to_reachable_set[time_step]:
            p_x_min, v_x_min, p_x_max, v_x_max = node.polygon_lon.bounds
            p_y_min, v_y_min, p_y_max, v_y_max = node.polygon_lat.bounds

            p_x_min_translated = p_x_min + p_translate[0]
            p_x_max_translated = p_x_max + p_translate[0]
            p_y_min_translated = p_y_min + p_translate[1]
            p_y_max_translated = p_y_max + p_translate[1]
            v_x_min_translated = max(v_x_min + v_translate[0], -self.config.vehicle.ego.v_max)
            v_x_max_translated = min(v_x_max + v_translate[0], self.config.vehicle.ego.v_max)
            v_y_min_translated = max(v_y_min + v_translate[1], -self.config.vehicle.ego.v_max)
            v_y_max_translated = min(v_y_max + v_translate[1], self.config.vehicle.ego.v_max)

            node.polygon_lon = ReachPolygon.from_rectangle_vertices(p_x_min_translated, v_x_min_translated,
                                                                    p_x_max_translated, v_x_max_translated)
            node.polygon_lat = ReachPolygon.from_rectangle_vertices(p_y_min_translated, v_y_min_translated,
                                                                    p_y_max_translated, v_y_max_translated)

    def _discard_invalid_reachable_set(self, time_step: int):
        """Discards invalid nodes of the reachable set.

        Nodes that are either colliding with obstacles, has no parent, or has no grandparent will be discarded.
        """
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

    def _prune_nodes_not_reaching_final_time_step(self):
        """Prunes nodes not reaching the final time step.

        Iterates through reachable sets in backward direction and discard nodes not reaching the final time step.
        """
        cnt_nodes_before_pruning = cnt_nodes_after_pruning = len(self.reachable_set_at_time_step(self.time_step_end))

        for time_step in range(self.time_step_end - 1, self.time_step_start - 1, -1):
            list_nodes = self.reachable_set_at_time_step(time_step)
            cnt_nodes_before_pruning += len(list_nodes)

            list_idx_nodes_to_be_deleted = list()
            for idx_node, node in enumerate(list_nodes):
                # discard if the node has no child node
                if not node.list_nodes_child:
                    list_idx_nodes_to_be_deleted.append(idx_node)
                    # iterate through parent nodes and disconnect them
                    for node_parent in node.list_nodes_parent:
                        node_parent.remove_child_node(node)

            self._dict_time_to_reachable_set[time_step] = [node for idx_node, node in enumerate(list_nodes)
                                                           if idx_node not in list_idx_nodes_to_be_deleted]
            self._dict_time_to_drivable_area[time_step] = [node.position_rectangle
                                                           for idx_node, node in enumerate(list_nodes)
                                                           if idx_node not in list_idx_nodes_to_be_deleted]

            cnt_nodes_after_pruning += len(list_nodes)

        self._pruned = True

        message = f"\t#Nodes before pruning: \t{cnt_nodes_before_pruning}"
        print(message)
        logger.info(message)

        message = f"\t#Nodes after pruning: \t{cnt_nodes_after_pruning}"
        print(message)
        logger.info(message)
