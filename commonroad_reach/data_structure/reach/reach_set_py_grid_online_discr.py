import logging
import math
from functools import lru_cache
from typing import Optional, List, Dict

from commonroad.geometry.shape import Rectangle, Polygon
from commonroad.scenario.scenario import Scenario
from commonroad_reach.data_structure.collision_checker_cpp import CppCollisionChecker
from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility.obstacle_grid import ObstacleRegularGrid
from scipy import sparse

logger = logging.getLogger(__name__)
import os
import pickle
import numpy as np
from collections import defaultdict

from commonroad_reach.data_structure.reach.reach_set import ReachableSet
from commonroad_reach.data_structure.collision_checker_py import PyCollisionChecker
from commonroad_reach.data_structure.configuration import Configuration


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
        self._collision_checker: Optional[CppCollisionChecker] = None

        self._initialize_collision_checker()
        self.occupied_grid_obs_static = ()
        self._reachability_grid: Dict[int, np.ndarray] = {}

        self.obstacle_grid: ObstacleRegularGrid = None
        self.dict_time_to_list_tuples_reach_node_attributes = {}
        self.dict_time_to_adjacency_matrices_parent = {}
        self.dict_time_to_adjacency_matrices_grandparent = {}

        self.reachset_bb_ll: Dict[int, np.ndarray] = dict()
        self.reachset_bb_ur: Dict[int, np.ndarray] = dict()
        self._dict_time_to_reachable_set_all: Dict[int, List[ReachNodeMultiGeneration]] = defaultdict(list)
        self._dict_time_to_drivable_area_all: Dict[int, List[ReachPolygon]] = defaultdict(list)

        self._restore_reachability_graph()
        self._initialize_collision_checker()

        self.initialize_new_scenario(self.config.scenario, self.config.planning_problem)

    def _dict_time_to_reachable_set(self):
        dict_time_to_reachable_set = {}
        for t in self._list_time_steps_computed:
            dict_time_to_reachable_set[t] = self.reachable_set_at_time_step(t)

        return dict_time_to_reachable_set

    @lru_cache(128)
    def reachable_set_at_time_step(self, time_step: int):
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for drivable area retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            reachset_list = self._dict_time_to_reachable_set_all[time_step]
            return [reachset_list[index_reachset]
                    for index_reachset, reachable in enumerate(self._reachability_grid[time_step].flatten())
                    if reachable]

    def _dict_time_to_drivable_area(self):
        dict_time_to_drivable_area = {}
        for t in self._list_time_steps_computed:
            dict_time_to_drivable_area[t] = self.drivable_area_at_time_step(t)

        return dict_time_to_drivable_area

    @lru_cache(128)
    def drivable_area_at_time_step(self, time_step: int):
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for drivable area retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            rectangle_list_all = self._dict_time_to_drivable_area_all[time_step]
            drivable_area = []
            for index_reachset, reachable in enumerate(self._reachability_grid[time_step].flatten()):
                if reachable:
                    try:
                        vertices = rectangle_list_all[index_reachset].vertices
                        vertices += self.reachset_translation(time_step)
                        drivable_area.append(ReachPolygon(vertices, fix_vertices=False))
                    except:
                        continue
            return drivable_area

    @property
    def max_evaluated_time_step(self):
        return max(self._reachability_grid)

    @lru_cache(128)
    def time_step(self, time_index: int) -> int:
        """
        Convert relative time index (initial time_index = 0) to time_step (initial time_step = time_step_start)
        :param time_index:
        :return:
        """
        return time_index + self.time_step_start

    def _restore_reachability_graph(self):
        """Restores reachability graph from the offline computation result."""
        self.dict_time_to_list_tuples_reach_node_attributes, self.dict_time_to_adjacency_matrices_parent, \
        self.dict_time_to_adjacency_matrices_grandparent, \
        self.reachset_bb_ll, self.reachset_bb_ur = self._load_offline_computation_result()
        self._num_time_steps_offline_computation = len(self.dict_time_to_list_tuples_reach_node_attributes)

        self._grid_shapes = {t: (round((self.reachset_bb_ur[t][0] - ll[0]) / self.config.reachable_set.size_grid),
                                 round((self.reachset_bb_ur[t][1] - ll[1]) / self.config.reachable_set.size_grid))
                             for t, ll in self.reachset_bb_ll.items()}
        self._restore_reachable_sets(self.dict_time_to_list_tuples_reach_node_attributes,
                                     self.dict_time_to_adjacency_matrices_parent,
                                     self.dict_time_to_adjacency_matrices_grandparent)

    def _load_offline_computation_result(self):
        """Loads pickle file generated in the offline computation step."""
        message = "* Loading offline computation result..."
        print(message)
        logger.info(message)

        path_file_pickle = os.path.join(self.config.general.path_offline_data,
                                        self.config.reachable_set.name_pickle_offline)
        dict_data = pickle.load(open(path_file_pickle, "rb"))

        return dict_data["node_attributes"], dict_data["adjacency_matrices_parent"], \
               dict_data["adjacency_matrices_grandparent"], \
               dict_data["reachset_bb_ll"], dict_data["reachset_bb_ur"]

    def _restore_reachable_sets(self, dict_time_to_list_tuples_reach_node_attributes,
                                dict_time_to_adjacency_matrices_parent,
                                dict_time_to_adjacency_matrices_grandparent):
        """Restores reachable sets from the offline computation result."""
        # todo: change to vertices of polytopes?
        for time_step, list_tuples_attribute in dict_time_to_list_tuples_reach_node_attributes.items():
            # reconstruct nodes in the reachability graph
            for tuple_attribute in list_tuples_attribute:
                p_x_min, p_y_min, p_x_max, p_y_max, v_x_min, v_y_min, v_x_max, v_y_max = tuple_attribute
                polygon_x = ReachPolygon.from_rectangle_vertices(p_x_min, v_x_min, p_x_max, v_x_max)
                polygon_y = ReachPolygon.from_rectangle_vertices(p_y_min, v_y_min, p_y_max, v_y_max)

                node = ReachNodeMultiGeneration(polygon_x, polygon_y, time_step)
                self._dict_time_to_reachable_set_all[time_step].append(node)
                position_rectangle = ReachPolygon.from_rectangle_vertices(p_x_min, p_y_min, p_x_max, p_y_max)
                self._dict_time_to_drivable_area_all[time_step].append(position_rectangle)
            # # restore parent-child relationship
            # if time_step >= 1:
            #     matrix_adjacency_dense = dict_time_to_adjacency_matrices_parent[time_step].todense()
            #     list_nodes_parent = self._dict_time_to_reachable_set_all[time_step - 1]
            #     list_nodes = self._dict_time_to_reachable_set_all[time_step]
            #
            #     for idx_node, list_adjacency in enumerate(matrix_adjacency_dense):
            #         node = list_nodes[idx_node]
            #         [list_adjacency] = list_adjacency.tolist()
            #
            #         for idx_parent, adjacency in enumerate(list_adjacency):
            #             node_parent = list_nodes_parent[idx_parent]
            #             if adjacency:
            #                 node.add_parent_node(node_parent)
            #                 node_parent.add_child_node(node)
            #
            # # restore grandparent-grandchild relationship
            # if time_step >= 2:
            #     matrix_adjacency_dense = dict_time_to_adjacency_matrices_grandparent[time_step].todense()
            #     list_nodes_grandparent = self._dict_time_to_reachable_set_all[time_step - 2]
            #     list_nodes = self._dict_time_to_reachable_set_all[time_step]
            #
            #     for idx_node, list_adjacency in enumerate(matrix_adjacency_dense):
            #         node = list_nodes[idx_node]
            #         [list_adjacency] = list_adjacency.tolist()
            #
            #         for idx_grandparent, adjacency in enumerate(list_adjacency):
            #             node_grandparent = list_nodes_grandparent[idx_grandparent]
            #             if adjacency:
            #                 node.add_grandparent_node(node_grandparent)
            #                 node_grandparent.add_grandchild_node(node)

    def initialize_new_scenario(self, scenario: Optional[Scenario], planning_problem: [Optional]):
        """
        Reset online computation for evaluation of new scenario and/or planning problem;
        thus, avoid time for parsing pickle file again.
        :param scenario: new scenario (keep old scenario if None)
        :param planning_problem: new planning_problem (keep old planning_problem if None)
        :return: None
        """
        update_planning_problem = planning_problem is not None
        update_cc = update_planning_problem or (scenario is not None)
        if scenario is not None:
            self.config.scenario = scenario

        if planning_problem is not None:
            self.config.planning_problem = planning_problem
            self.reachset_translation.cache_clear()

        if update_cc:
            self._initialize_collision_checker()
            self._occ_grid_at_time.cache_clear()

            self.obstacle_grid = ObstacleRegularGrid \
                (self.reachset_bb_ll, self.reachset_bb_ur, self._collision_checker.collision_checker,
                 self.config.reachable_set.size_grid, self.config.reachable_set.size_grid,
                 self.config.planning_problem,
                 a_x=self.config.vehicle.ego.a_max, a_y=self.config.vehicle.ego.a_max,
                 t_f=self.config.scenario.dt * self._num_time_steps_offline_computation,
                 grid_shapes=self._grid_shapes)

        self.occupied_grid_obs_static = dict()
        self._reachability_grid.clear()
        self.reachable_set_at_time_step.clear()
        self.drivable_area_at_time_step.clear()
        self.time_step.clear()

        self._reachability_grid[self.time_step_start] = np.ones((1, 1), dtype=bool)

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

    def compute_reachable_sets(self, time_step_start: int, time_step_end: int, n_multisteps: int = 0):
        for time_step in range(time_step_start, time_step_end):
            if time_step > self._num_time_steps_offline_computation:
                message = f"Time step {time_step} is out of range, max allowed: {self._num_time_steps_offline_computation}"
                print(message)
                logger.warning(message)
                return

            self._list_time_steps_computed.append(time_step)
            self._forward_propagation(time_step, n_multisteps)
            self._list_time_steps_computed.append(time_step + 1)

        if self.config.reachable_set.prune_nodes_not_reaching_final_time_step:
            self._prune_nodes_not_reaching_final_time_step()

    def _prune_nodes_not_reaching_final_time_step(self):
        """Prunes nodes that don't reach the final time step.

        Iterates through reachability graph backward in time, discards nodes that don't have a child node.
        """
        for i_t in range(self.max_evaluated_time_step, 0, -1):
            self.backward_step(i_t)

    @lru_cache(124)
    def reachset_translation(self, time_step: int):
        """Translation of initial state at time_step"""
        initial_state = self.config.planning_problem.initial_state
        v_init = np.array([math.cos(initial_state.orientation),
                           math.sin(initial_state.orientation)]) * initial_state.velocity
        return initial_state.position + v_init * self.config.scenario.dt * time_step

    def _forward_propagation(self, init_time_step: int, n_multisteps: int, repeated=False):
        """
        Propagates current reachability grid and excludes forbidden states
        :param n_multisteps:
        :param reachset_translation_tmp: translation of reachset center at current time step
        :return:
        """
        if init_time_step >= self._num_time_steps_offline_computation:
            raise ValueError('Reached max number of offline computed time steps!')
        try:
            reachability_grid_prop = self.dict_time_to_adjacency_matrices_parent[init_time_step + 1].dot(
                self._reachability_grid[init_time_step].reshape([-1, 1]))
        except:
            raise

        if sparse.issparse(reachability_grid_prop):
            reachability_grid_prop = reachability_grid_prop.toarray()

        # propagate grandparents:
        if False and init_time_step + 1 in self.dict_time_to_adjacency_matrices_grandparent:
            for time_step_gp, adj_matrix_gp in self.dict_time_to_adjacency_matrices_grandparent[
                init_time_step + 1].items():  # get time index of grandparent to propagate
                delta_time_step = init_time_step - time_step_gp
                if delta_time_step < n_multisteps and time_step_gp in self._reachability_grid:
                    reachability_grid_prop_grandparent = \
                        adj_matrix_gp.dot(self._reachability_grid[time_step_gp].reshape([-1, 1]))
                    if sparse.issparse(reachability_grid_prop_grandparent):
                        prop_grandparent_tmp = reachability_grid_prop_grandparent.toarray()
                    else:
                        prop_grandparent_tmp = reachability_grid_prop_grandparent

                    # intersect with propagated cells of other time steps
                    reachability_grid_prop = np.logical_and(reachability_grid_prop, prop_grandparent_tmp)

        # intersect propagated cells with occupied cells
        try:
            reachability_grid_prop = np.logical_and(reachability_grid_prop.reshape([-1, 1]),
                                                    self._occ_grid_at_time(init_time_step + 1))
        except:
            raise

        self._reachability_grid[init_time_step + 1] = reachability_grid_prop

    @lru_cache(128)
    def _occ_grid_at_time(self, time_step: int):
        occupied_grid_obs = self.obstacle_grid.occupancy_grid_at_time(time_step, self.reachset_translation(time_step))
        return occupied_grid_obs.reshape([-1, 1])

    def backward_step(self, init_time_step: int):

        if init_time_step <= 0:
            logger.warning('Reached max number of backward timesteps')
            return

        try:
            reachability_grid_prop = self.dict_time_to_adjacency_matrices_parent[init_time_step].transpose() * \
                                     self._reachability_grid[init_time_step].reshape([-1, 1])
        except:
            raise
        if sparse.issparse(reachability_grid_prop):
            reachability_grid_prop = reachability_grid_prop.toarray()

        # intersect propagated cells with occupied cells
        reachability_grid_prop_pruned = np.logical_and(reachability_grid_prop.reshape([-1, 1]),
                                                       self._occ_grid_at_time(init_time_step - 1))

        self._reachability_grid[init_time_step - 1] = \
            np.logical_and(self._reachability_grid[init_time_step - 1], reachability_grid_prop_pruned)
