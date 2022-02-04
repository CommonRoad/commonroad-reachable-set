import logging
import os
import pickle
import time
from typing import List, Dict

import numpy as np
from scipy import sparse

from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.data_structure.reach.reach_set import ReachableSet

logger = logging.getLogger(__name__)

from collections import defaultdict

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.utility import reach_operation


class PyGridOfflineReachableSet(ReachableSet):
    """Offline step in the multi-step reachable set computation with Python backend."""

    def __init__(self, config: Configuration):
        super().__init__(config)

        if config.planning.coordinate_system != "CART":
            message = "Multi-step reachable set computation only supports Cartesian coordinate system."
            logger.error(message)
            raise Exception(message)

        self.polygon_zero_state_lon = None
        self.polygon_zero_state_lat = None
        self._initialize_zero_state_polygons()
        self._dict_time_to_drivable_area[self.time_step_start] = self.initial_drivable_area
        self._dict_time_to_reachable_set[self.time_step_start] = self.initial_reachable_set

    def _initialize_zero_state_polygons(self):
        """Initializes the zero-state polygons of the system.

        Computation of the reachable set of an LTI system requires the zero-state response and the zero-input response
        of the system.
        """
        self.polygon_zero_state_lon = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                -self.config.vehicle.ego.a_max,
                                                                                self.config.vehicle.ego.a_max)

        self.polygon_zero_state_lat = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                -self.config.vehicle.ego.a_max,
                                                                                self.config.vehicle.ego.a_max)

    @property
    def initial_drivable_area(self) -> List[ReachPolygon]:
        """Drivable area at the initial time step.

        Constructed directly from the config file.
        """
        tuple_vertices = reach_operation.generate_tuple_vertices_position_rectangle_initial(self.config)

        return [ReachPolygon.from_rectangle_vertices(*tuple_vertices)]

    @property
    def initial_reachable_set(self) -> List[ReachNodeMultiGeneration]:
        """Reachable set at the initial time step.

        Vertices of the polygons are constructed directly from the config file.
        """
        tuple_vertices_polygon_lon, tuple_vertices_polygon_lat = \
            reach_operation.generate_tuples_vertices_polygons_initial(self.config)

        polygon_lon = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lon)
        polygon_lat = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lat)

        return [ReachNodeMultiGeneration(polygon_lon, polygon_lat, self.config.planning.time_step_start)]

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

        self._determine_grandparent_relationship(self._dict_time_to_reachable_set)

        # save computation result to pickle file
        self._save_to_pickle()

    def _compute_at_time_step(self, time_step: int):
        """Computes drivable area and reachable set of the time step."""
        self._compute_drivable_area_at_time_step(time_step)
        self._compute_reachable_set_at_time_step(time_step)

    def _compute_drivable_area_at_time_step(self, time_step: int):
        """Computes the drivable area for the given time step.

        Steps:
            1. Propagate each node of the reachable set from the previous time step, resulting in propagated base sets.
            2. Project the base sets onto the position domain to obtain the position rectangles.
            3. Merge and repartition these rectangles to potentially reduce the number of rectangles.
            4. Adapt position rectangles to Cartesian grid.
            5. Remove rectangles out of Kamm's friction circle.
        """
        reachable_set_previous = self._dict_time_to_reachable_set[time_step - 1]
        if len(reachable_set_previous) < 1:
            return [], []

        size_grid = self.config.reachable_set.size_grid

        list_base_sets_propagated = self._propagate_reachable_set(reachable_set_previous)

        list_rectangles_projected = reach_operation.project_base_sets_to_position_domain(list_base_sets_propagated)

        list_rectangles_repartitioned = \
            reach_operation.create_repartitioned_rectangles(list_rectangles_projected, size_grid)

        list_rectangles_adapted = reach_operation.adapt_rectangles_to_grid(list_rectangles_repartitioned, size_grid)

        drivable_area = \
            reach_operation.remove_rectangles_out_of_kamms_circle(time_step * self.config.planning.dt,
                                                                  self.config.vehicle.ego.a_max,
                                                                  list_rectangles_adapted)

        self._dict_time_to_drivable_area[time_step] = drivable_area
        self._dict_time_to_base_set_propagated[time_step] = list_base_sets_propagated

    def _propagate_reachable_set(self, list_nodes: List[ReachNodeMultiGeneration]) -> List[ReachNodeMultiGeneration]:
        """Propagates the nodes of the reachable set from the previous time step."""
        list_base_sets_propagated = []

        for node in list_nodes:
            try:
                polygon_lon_propagated = reach_operation.propagate_polygon(node.polygon_lon,
                                                                           self.polygon_zero_state_lon,
                                                                           self.config.planning.dt,
                                                                           -self.config.vehicle.ego.v_max,
                                                                           self.config.vehicle.ego.v_max)

                polygon_lat_propagated = reach_operation.propagate_polygon(node.polygon_lat,
                                                                           self.polygon_zero_state_lat,
                                                                           self.config.planning.dt,
                                                                           -self.config.vehicle.ego.v_max,
                                                                           self.config.vehicle.ego.v_max)
            except (ValueError, RuntimeError):
                logger.warning("Error occurred while propagating polygons.")

            else:
                base_set_propagated = ReachNodeMultiGeneration(polygon_lon_propagated, polygon_lat_propagated,
                                                               node.time_step)
                base_set_propagated.source_propagation = node
                list_base_sets_propagated.append(base_set_propagated)

        return list_base_sets_propagated

    def _compute_reachable_set_at_time_step(self, time_step):
        """Computes the reachable set for the given time step.

        Steps:
            1. create a list of base sets adapted to the drivable area.
            2. create a list of reach nodes from the list of adapted base sets.
        """
        base_sets_propagated = self._dict_time_to_base_set_propagated[time_step]
        drivable_area = self._dict_time_to_drivable_area[time_step]
        if not drivable_area:
            return []

        list_base_sets_adapted = reach_operation.adapt_base_sets_to_drivable_area(drivable_area,
                                                                                  base_sets_propagated,
                                                                                  has_multi_generation=True)

        reachable_set_time_step_current = reach_operation.create_nodes_of_reachable_set(time_step,
                                                                                        list_base_sets_adapted)

        self._dict_time_to_reachable_set[time_step] = reachable_set_time_step_current

    def _determine_grandparent_relationship(self,
                                            dict_time_to_reachable_set: Dict[int, List[ReachNodeMultiGeneration]]):
        """Determines grandparent-child relationship between nodes.

        The grandparent-child relationship is established if the grandparent can reach the grandchild by propagating
        two time steps.
        """
        logger.debug("Determining grandparent-grandchild relationship...")
        for time_step in list(dict_time_to_reachable_set.keys())[2:]:
            list_nodes = dict_time_to_reachable_set[time_step]
            list_nodes_grand_parent = dict_time_to_reachable_set[time_step - 2]
            list_nodes_grand_parent_propagated = list_nodes_grand_parent

            for _ in range(2):
                list_nodes_grand_parent_propagated = self._propagate_reachable_set(list_nodes_grand_parent_propagated)

            for node in list_nodes:
                rectangle_node = node.position_rectangle

                for idx_grandparent, node_grand_parent_propagated in enumerate(list_nodes_grand_parent_propagated):
                    rectangle_node_grand_parent_propagated = node_grand_parent_propagated.position_rectangle
                    node_grand_parent = list_nodes_grand_parent[idx_grandparent]

                    if rectangle_node.intersects(rectangle_node_grand_parent_propagated):
                        node.add_grandparent_node(node_grand_parent)
                        node_grand_parent.add_grandchild_node(node)

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

    def _prune_nodes_not_reaching_final_time_step(self):
        pass
