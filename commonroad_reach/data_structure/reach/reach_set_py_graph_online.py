import pyximport

pyximport.install()

import os
import pickle
import logging
import warnings
from functools import lru_cache
from collections import defaultdict
from typing import Optional, List, Dict, Tuple

import numpy as np
from scipy import sparse

from commonroad.scenario.scenario import Scenario
from commonroad_reach.__version__ import __version__
from commonroad_reach.data_structure.collision_checker import CollisionChecker
from commonroad_reach.data_structure.configuration import Configuration, VehicleConfiguration, ReachableSetConfiguration
from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration, ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.data_structure.reach.reach_set import ReachableSet
from commonroad_reach.data_structure.regular_grid import RegularGrid
import commonroad_reach.utility.logger as util_logger

logger = logging.getLogger("REACH_LOGGER")


class PyGraphReachableSetOnline(ReachableSet):
    """
    Online step in the graph-based reachable set computation with Python backend.
    """

    def __init__(self, config: Configuration):
        super().__init__(config)

        self._num_time_steps_offline_computation = 0
        self._collision_checker: Optional[CollisionChecker] = None

        self._reachability_grid: Dict[int, np.ndarray] = {}
        self.obstacle_grid: Optional[RegularGrid] = None
        self.dict_time_to_list_tuples_reach_node_attributes = {}
        self.dict_time_to_adjacency_matrices_parent = {}
        self.dict_time_to_adjacency_matrices_grandparent = {}
        self.reachset_bb_ll: Dict[int, np.ndarray] = dict()
        self.reachset_bb_ur: Dict[int, np.ndarray] = dict()

        # contains all pre-computed nodes
        self._dict_time_to_reachable_set_all: Dict[int, List[ReachNodeMultiGeneration]] = defaultdict(list)
        self._dict_time_to_drivable_area_all: Dict[int, List[ReachPolygon]] = defaultdict(list)

        self._restore_reachability_graph()
        self._initialize_collision_checker()
        self.initialize_new_scenario(self.config.scenario, self.config.planning_problem)

        logger.info("PyGraphReachableSetOnline initialized.")

    @property
    def max_evaluated_step(self) -> int:
        return max(self._reachability_grid)

    def _dict_step_to_drivable_area(self) -> Dict[int, List[ReachPolygon]]:
        dict_step_to_drivable_area = {}
        for t in self._list_steps_computed:
            dict_step_to_drivable_area[t] = self.drivable_area_at_step(t)

        return dict_step_to_drivable_area

    def _dict_step_to_reachable_set(self) -> Dict[int, List[ReachNodeMultiGeneration]]:
        dict_step_to_reachable_set = {}
        for t in self._list_steps_computed:
            dict_step_to_reachable_set[t] = self.reachable_set_at_step(t)

        return dict_step_to_reachable_set

    @lru_cache(128)
    def _occ_grid_at_step(self, step: int) -> np.ndarray:
        occupied_grid_obs = self.obstacle_grid.occupancy_grid_at_step(step, self.reachset_translation(step))
        return occupied_grid_obs.reshape([-1, 1])

    @lru_cache(128)
    def drivable_area_at_step(self, step: int) -> List[ReachPolygon]:
        if step not in self._list_steps_computed:
            util_logger.print_and_log_warning(logger, f"Given step {step} for drivable area retrieval is out of range.")
            return []

        else:
            rectangle_list_all = self._dict_time_to_drivable_area_all[step]
            drivable_area = []
            for index_reachset, reachable in enumerate(self._reachability_grid[step].flatten()):
                if reachable:
                    try:
                        vertices = rectangle_list_all[index_reachset].vertices
                        vertices += self.reachset_translation(step)
                        drivable_area.append(ReachPolygon(vertices, fix_vertices=False))

                    except:
                        continue
            return drivable_area

    @lru_cache(128)
    def reachable_set_at_step(self, step: int) -> List[ReachNodeMultiGeneration]:
        if step not in self._list_steps_computed:
            util_logger.print_and_log_warning(logger, "Given step for drivable area retrieval is out of range.")
            return []

        else:
            reachset_list_all = self._dict_time_to_reachable_set_all[step]
            reachset = []
            for index_reachset in np.argwhere(self._reachability_grid[step]):
                reachset.append(reachset_list_all[index_reachset[0]].
                                translate(p_lon_off=self.reachset_translation(step)[0],
                                          p_lat_off=self.reachset_translation(step)[1],
                                          v_lon_off=self.config.planning.v_lon_initial))

            if step > 0:
                self._restore_parent_node_relationships(reachset, step)

            return reachset

    @lru_cache(128)
    def time_step(self, time_index: int) -> int:
        """
        Converts relative time index (initial time_index = 0) to time_step (initial step = step_start)
        """
        return time_index + self.step_start

    @lru_cache(128)
    def reachset_translation(self, step: int) -> np.ndarray:
        """
        Translation of initial state at the given step.
        """
        return self.config.planning.p_initial \
               + self.config.planning.v_initial * self.config.scenario.dt * step

    def initialize_new_scenario(self, scenario: Optional[Scenario] = None, planning_problem: [Optional] = None):
        """
        Resets online computation for evaluation of new scenario and/or planning problem;
        thus, avoid time for parsing pickle file again.

        :param scenario: new scenario (keep old scenario if None)
        :param planning_problem: new planning_problem (keep old planning_problem if None)
        """
        update_planning_problem = planning_problem is not None
        update_cc = update_planning_problem or (scenario is not None)
        if scenario is not None:
            self.config.scenario = scenario

        if planning_problem is not None:
            self.config.update(scenario, planning_problem=planning_problem)
            self.reachset_translation.cache_clear()

        if update_cc:
            self._initialize_collision_checker()

            self.obstacle_grid = RegularGrid(self.reachset_bb_ll, self.reachset_bb_ur,
                                             self._collision_checker.cpp_collision_checker,
                                             self.config.reachable_set.size_grid, self.config.reachable_set.size_grid,
                                             self.config.planning,
                                             a_lon=self.config.vehicle.ego.a_max, a_lat=self.config.vehicle.ego.a_max,
                                             t_f=self.config.scenario.dt * self._num_time_steps_offline_computation,
                                             grid_shapes=self._grid_shapes)

        self._reachability_grid.clear()
        self.reachable_set_at_step.cache_clear()
        self.drivable_area_at_step.cache_clear()
        self._occ_grid_at_step.cache_clear()
        self._list_steps_computed = [0]
        self._reachability_grid[self.step_start] = np.ones((1, 1), dtype=bool)

    def _restore_parent_node_relationships(self, reachset: List[ReachNode], step: int):
        """
        Restores parent-child relationships.
        """
        if step == 0:
            return

        ind_2_list_index_prev = np.insert(np.cumsum(self._reachability_grid[step - 1]), 0, 0)
        ind_2_list_index_current = np.insert(np.cumsum(self._reachability_grid[step]), 0, 0)
        for index_reachset in np.flatnonzero(self._reachability_grid[step]):
            reachable_parents = np.asarray(np.logical_and(self._reachability_grid[step - 1].flatten(),
                                                          self.dict_time_to_adjacency_matrices_parent[
                                                              step].todense()[
                                                          index_reachset, :]))
            node = reachset[ind_2_list_index_current[index_reachset]]
            for index_parent in np.flatnonzero(reachable_parents):
                try:
                    parent = self.reachable_set_at_step(step - 1)[ind_2_list_index_prev[index_parent] - 1]
                    parent.add_child_node(node)
                    node.add_parent_node(parent)
                except IndexError:
                    continue

    def _restore_reachability_graph(self):
        """
        Restores reachability graph from the offline computation result.
        """
        self.dict_time_to_list_tuples_reach_node_attributes, self.dict_time_to_adjacency_matrices_parent, \
        self.dict_time_to_adjacency_matrices_grandparent, \
        self.reachset_bb_ll, self.reachset_bb_ur = self._load_offline_computation_result()
        self._num_time_steps_offline_computation = len(self.dict_time_to_list_tuples_reach_node_attributes)

        self._grid_shapes = {t: (round((self.reachset_bb_ur[t][0] - ll[0]) / self.config.reachable_set.size_grid),
                                 round((self.reachset_bb_ur[t][1] - ll[1]) / self.config.reachable_set.size_grid))
                             for t, ll in self.reachset_bb_ll.items()}
        self._restore_reachable_sets(self.dict_time_to_list_tuples_reach_node_attributes)

    def _load_offline_computation_result(self) -> Tuple:
        """
        Loads pickle file generated in the offline computation step.
        """
        util_logger.print_and_log_info(logger, "* Loading offline computation result...")

        path_file_pickle = os.path.join(self.config.general.path_offline_data,
                                        self.config.reachable_set.name_pickle_offline)
        dict_data = pickle.load(open(path_file_pickle, "rb"))
        if dict_data["__version__"] != __version__:
            raise ValueError(f"Offline data was created with an older version of commonroad-reach "
                             f"{dict_data['__version__']}. "
                             f"Please recreate the file with the current version {__version__} !")
        assert dict_data["coordinate_system"] == self.config.planning.coordinate_system, \
            f"pickle file was created for coordinate_system={dict_data['coordinate_system']}," \
            f"not {self.config.planning.coordinate_system}!"

        self._validate_configurations(self.config.reachable_set, self.config.vehicle,
                                      dict_data["config.reachable_set"], dict_data["config.vehicle"])
        return dict_data["node_attributes"], dict_data["adjacency_matrices_parent"], \
               dict_data["adjacency_matrices_grandparent"], \
               dict_data["reachset_bb_ll"], dict_data["reachset_bb_ur"]

    @staticmethod
    def _validate_configurations(reachset_config_online: ReachableSetConfiguration,
                                 vehicle_config_online: VehicleConfiguration,
                                 reachset_config_offline: ReachableSetConfiguration,
                                 vehicle_config_offline: VehicleConfiguration):
        """
        Ensures that original configuration from the offline data is used for relevant parameters.
        """

        def validate_and_update_config(config_online, config_offline, relevant_attributes):
            for attr in vars(config_online):
                if attr in relevant_attributes and hasattr(config_offline, attr):
                    if getattr(config_online, attr) != getattr(config_offline, attr):
                        online_value = getattr(config_online, attr)
                        offline_value = getattr(config_offline, attr)
                        warn_text = f"Parameter {config_online.__class__.__name__}.{attr}=" \
                                    f"{online_value}!={offline_value}, which was " \
                                    f"used to create {reachset_config_online.name_pickle_offline}. Overwriting value..."
                        warnings.warn(warn_text)
                        logger.warning(warn_text)
                        setattr(config_online, attr, offline_value)

        relevant_attributes_reachset = \
            ["size_grid"]
        relevant_attributes_ego_vehicle = \
            ["a_lon_max",
             "a_lon_min",
             "a_lat_max",
             "a_lat_min",
             "a_max",
             "v_lon_max"]

        validate_and_update_config(reachset_config_online, reachset_config_offline, relevant_attributes_reachset)
        validate_and_update_config(vehicle_config_online.ego, vehicle_config_offline.ego,
                                   relevant_attributes_ego_vehicle)

        assert reachset_config_online.n_multi_steps <= reachset_config_offline.n_multi_steps, \
            f"pre-computed only {reachset_config_offline.n_multi_steps} multi-steps " \
            f"but requested {reachset_config_online.n_multi_steps}"

    def _restore_reachable_sets(self, dict_time_to_list_tuples_reach_node_attributes: Dict[int, Tuple]):
        """
        Restores reachable sets from the offline computation result.
        """
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

    def _initialize_collision_checker(self):
        self._collision_checker = CollisionChecker(self.config)

    def compute(self, step_start: int = 1, step_end: Optional[int] = None):
        if step_end is None:
            step_end = self.step_end

        for step in range(step_start, step_end + 1):
            if step > self._num_time_steps_offline_computation:
                util_logger.print_and_log_warning(logger, f"Time step {step} is out of range, max allowed: "
                                                          f"{self._num_time_steps_offline_computation}")
                return

            self._forward_propagation(step, self.config.reachable_set.n_multi_steps)
            self._list_steps_computed.append(step)

        if self.config.reachable_set.prune_nodes_not_reaching_final_step:
            self.prune_nodes_not_reaching_final_step()

    def _forward_propagation(self, step: int, n_multi_steps: int):
        """
        Propagates current reachability grid and excludes forbidden states.

        :param step: initial step of the reachable set
        :param n_multi_steps: number of previous time steps considered for the propagation
        """
        if step >= self._num_time_steps_offline_computation:
            raise ValueError(f'Reached max number of offline computed time steps '
                             f'({self._num_time_steps_offline_computation})!')

        reachability_grid_prop = self.dict_time_to_adjacency_matrices_parent[step].dot(
            self._reachability_grid[step - 1].reshape([-1, 1]))

        if sparse.issparse(reachability_grid_prop):
            reachability_grid_prop = reachability_grid_prop.toarray()

        # propagate grandparents:
        if step > 1:
            for delta_time_step, adj_matrix_gp in self.dict_time_to_adjacency_matrices_grandparent[step].items():
                # get time index of grandparent to propagate
                step_gp = step - delta_time_step
                if delta_time_step <= n_multi_steps and step_gp in self._reachability_grid:
                    reachability_grid_prop_grandparent = \
                        adj_matrix_gp.dot(self._reachability_grid[step_gp].reshape([-1, 1]))
                    if sparse.issparse(reachability_grid_prop_grandparent):
                        prop_grandparent_tmp = reachability_grid_prop_grandparent.toarray()
                    else:
                        prop_grandparent_tmp = reachability_grid_prop_grandparent

                    # intersect with propagated cells of other time steps
                    reachability_grid_prop = np.logical_and(reachability_grid_prop, prop_grandparent_tmp)

            # intersect propagated cells with occupied cells
            reachability_grid_prop = np.logical_and(reachability_grid_prop.reshape([-1, 1]),
                                                    self._occ_grid_at_step(step))

        self._reachability_grid[step] = reachability_grid_prop

    def prune_nodes_not_reaching_final_step(self):
        """
        Prunes nodes that do not reach the final time step.
        """
        for i_t in range(self.max_evaluated_step - 1, 0, -1):
            self._backward_step(i_t)

        self._pruned = True
        self.reachable_set_at_step.cache_clear()
        self.drivable_area_at_step.cache_clear()

    def _backward_step(self, step: int):
        """
        Iterates through reachability graph backward in time, discards nodes that do not have a child node.
        """
        if step < 0:
            logger.warning('Reached max number of backward time steps')
            return

        reachability_grid_prop = self.dict_time_to_adjacency_matrices_parent[step + 1].transpose() * \
                                 self._reachability_grid[step + 1].reshape([-1, 1])
        if sparse.issparse(reachability_grid_prop):
            reachability_grid_prop = reachability_grid_prop.toarray()

        # intersect propagated cells with occupied cells
        reachability_grid_prop_pruned = np.logical_and(reachability_grid_prop.reshape([-1, 1]),
                                                       self._occ_grid_at_step(step))

        self._reachability_grid[step] = \
            np.logical_and(self._reachability_grid[step], reachability_grid_prop_pruned)

    def compute_drivable_area_at_step(self, step: int):
        pass

    def compute_reachable_set_at_step(self, step: int):
        pass

    def _reset_reachable_set_at_step(self, step: int, reachable_set):
        pass
