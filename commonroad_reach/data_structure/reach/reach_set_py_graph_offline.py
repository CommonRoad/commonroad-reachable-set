import os
import time
import pickle
import logging
from collections import defaultdict
from typing import List, Dict, Tuple

import math
import numpy as np
from scipy import sparse
from commonroad_dc.pycrcc import CollisionChecker, RectAABB

from commonroad_reach.__version__ import __version__
from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.data_structure.reach.reach_set import ReachableSet
from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.utility import reach_operation
import commonroad_reach.utility.logger as util_logger

logger = logging.getLogger(__name__)


class PyGraphReachableSetOffline(ReachableSet):
    """Offline step in the graph-based reachable set computation with Python backend."""

    def __init__(self, config: Configuration):
        self.dict_step_to_drivable_area: Dict[int, List[ReachPolygon]]
        self.dict_step_to_reachable_set: Dict[int, List[ReachNodeMultiGeneration]]
        super().__init__(config)

        self.polygon_zero_state_lon: Dict[int, ReachPolygon] = dict()
        self.polygon_zero_state_lat: Dict[int, ReachPolygon] = dict()
        self._initialize_zero_state_polygons()

        self.dict_step_to_drivable_area[self.step_start] = self.initial_drivable_area
        self.dict_step_to_reachable_set[self.step_start] = self.initial_reachable_set

        logger.info("PyGraphReachableSetOffline initialized.")

    @property
    def path_offline_file(self):
        if self.config.reachable_set.name_pickle_offline is None:
            return None
        return os.path.join(self.config.general.path_offline_data, self.config.reachable_set.name_pickle_offline)

    def _initialize_zero_state_polygons(self):
        """Initializes the zero-state polygons of the system.

        Computation of the reachable set of an LTI system requires the zero-state response and the zero-input response
        of the system.
        """
        for steps in range(1, self.config.reachable_set.n_multi_steps):
            dt = self.config.planning.dt * steps
            if self.config.planning.coordinate_system == "CART":
                a_lon_min = -self.config.vehicle.ego.a_max
                a_lon_max = self.config.vehicle.ego.a_max
                a_lat_min = -self.config.vehicle.ego.a_max
                a_lat_max = self.config.vehicle.ego.a_max
            else:
                a_lon_min = self.config.vehicle.ego.a_lon_min
                a_lon_max = self.config.vehicle.ego.a_lon_max
                a_lat_min = self.config.vehicle.ego.a_lat_min
                a_lat_max = self.config.vehicle.ego.a_lat_max

            self.polygon_zero_state_lon[steps] = reach_operation.create_zero_state_polygon(dt, a_lon_min, a_lon_max)
            self.polygon_zero_state_lat[steps] = reach_operation.create_zero_state_polygon(dt, a_lat_min, a_lat_max)

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

        return [ReachNodeMultiGeneration(polygon_lon, polygon_lat, self.config.planning.step_start)]

    def compute(self, step_start: int, step_end: int):
        """Computes reachable sets for the specified time steps."""
        for step in range(step_start + 1, step_end + 1):

            time_start = time.time()
            self._compute_at_step(step)
            self._list_steps_computed.append(step)

            util_logger.print_and_log_debug(logger, f"\t#Nodes: {len(self.reachable_set_at_step(step))}, " \
                                                    f"Took: {time.time() - time_start:.3f}s")

            if self.config.reachable_set.n_multi_steps >= 2:
                self._determine_grandparent_relationship(step)

            # save computation result to pickle file
            self._save_to_pickle()

    def _compute_at_step(self, step: int):
        """Computes drivable area and reachable set of the time step."""
        self._compute_drivable_area_at_step(step)
        self._compute_reachable_set_at_step(step)

    def _compute_drivable_area_at_step(self, step: int):
        """Computes the drivable area for the given time step.

        Steps:
            1. Propagate each node of the reachable set from the previous time step, resulting in propagated base sets.
            2. Project the base sets onto the position domain to obtain the position rectangles.
            3. Merge and repartition these rectangles to potentially reduce the number of rectangles.
            4. Adapt position rectangles to Cartesian grid.
            5. Remove rectangles out of Kamm's friction circle.
        """
        reachable_set_previous = self.dict_step_to_reachable_set[step - 1]
        if len(reachable_set_previous) < 1:
            return [], []

        size_grid = self.config.reachable_set.size_grid

        list_base_sets_propagated = self._propagate_reachable_set(reachable_set_previous)

        list_rectangles_projected = reach_operation.project_base_sets_to_position_domain(list_base_sets_propagated)

        list_rectangles_repartitioned = \
            reach_operation.create_repartitioned_rectangles(list_rectangles_projected, size_grid)

        list_rectangles_adapted = reach_operation.adapt_rectangles_to_grid(list_rectangles_repartitioned, size_grid)

        self.dict_step_to_drivable_area[step] = list_rectangles_adapted
        self.dict_step_to_base_set_propagated[step] = list_base_sets_propagated

    def _propagate_reachable_set(self, list_nodes: List[ReachNodeMultiGeneration], steps=1) -> List[
        ReachNodeMultiGeneration]:
        """Propagates the nodes of the reachable set from the previous time step."""
        assert steps >= 1
        list_base_sets_propagated = []
        dt = self.config.planning.dt * steps
        if self.config.planning.coordinate_system == "CART":
            v_lon_min = -self.config.vehicle.ego.v_max
            v_lon_max = self.config.vehicle.ego.v_max
            v_lat_min = -self.config.vehicle.ego.v_max
            v_lat_max = self.config.vehicle.ego.v_max
        else:
            v_lon_min = -self.config.vehicle.ego.v_lon_max
            v_lon_max = self.config.vehicle.ego.v_lon_max
            v_lat_min = -self.config.vehicle.ego.v_lat_max
            v_lat_max = self.config.vehicle.ego.v_lat_max

        for node in list_nodes:
            try:
                polygon_lon_propagated = reach_operation.propagate_polygon(node.polygon_lon,
                                                                           self.polygon_zero_state_lon[steps],
                                                                           dt,
                                                                           v_lon_min,
                                                                           v_lon_max)

                polygon_lat_propagated = reach_operation.propagate_polygon(node.polygon_lat,
                                                                           self.polygon_zero_state_lat[steps],
                                                                           dt,
                                                                           v_lat_min,
                                                                           v_lat_max)
            except (ValueError, RuntimeError):
                logger.warning("Error occurred while propagating polygons.")

            else:
                base_set_propagated = ReachNodeMultiGeneration(polygon_lon_propagated, polygon_lat_propagated,
                                                               node.step)
                base_set_propagated.source_propagation = node
                list_base_sets_propagated.append(base_set_propagated)

        return list_base_sets_propagated

    def _compute_reachable_set_at_step(self, step):
        """Computes the reachable set for the given time step.

        Steps:
            1. create a list of base sets adapted to the drivable area.
            2. create a list of reach nodes from the list of adapted base sets.
        """
        base_sets_propagated = self.dict_step_to_base_set_propagated[step]
        drivable_area = self.dict_step_to_drivable_area[step]
        if not drivable_area:
            return []

        list_base_sets_adapted = reach_operation.construct_reach_nodes(drivable_area,
                                                                       base_sets_propagated,
                                                                       has_multi_generation=True)

        reachable_set_step_current = reach_operation.connect_children_to_parents(step,
                                                                                 list_base_sets_adapted)

        self.dict_step_to_reachable_set[step] = reachable_set_step_current

    def _determine_grandparent_relationship(self, step: int):
        """Determines grandparent-child relationship between nodes.

        The grandparent-child relationship is established if the grandparent can reach the grandchild by propagating
        two time steps.
        """
        logger.debug("Determining grandparent-grandchild relationship...")
        cc_at_time: Dict[int, Tuple[CollisionChecker, Dict[RectAABB, ReachNodeMultiGeneration]]] = dict()

        def create_cc(nodes: List[ReachNodeMultiGeneration],
                      list_nodes_keys: List[ReachNodeMultiGeneration]) -> Tuple[CollisionChecker,
                                                                                Dict[
                                                                                    RectAABB, ReachNodeMultiGeneration]]:
            """Create collision checker from reachable sets."""
            cc = CollisionChecker()
            cc_obj_2_node = dict()
            for node, key_node in zip(nodes, list_nodes_keys):
                bounds = node.position_rectangle.bounds
                cc_obj = RectAABB(0.5 * (bounds[2] - bounds[0]),
                                  0.5 * (bounds[3] - bounds[1]),
                                  node.position_rectangle.p_lon_center,
                                  node.position_rectangle.p_lat_center)
                cc_obj_2_node[cc_obj] = key_node
                cc.add_collision_object(cc_obj)
            return cc, cc_obj_2_node

        def get_cc_at_time(time_step: int) -> Tuple[CollisionChecker, Dict[RectAABB, ReachNodeMultiGeneration]]:
            """Returns collision checker from reachable sets at time_step."""
            if time_step not in cc_at_time:
                cc_at_time[time_step] = create_cc(self.dict_step_to_reachable_set[time_step],
                                                  self.dict_step_to_reachable_set[time_step])
            return cc_at_time[time_step]

        for delta_steps in range(2, self.config.reachable_set.n_multi_steps):
            for step in list(self.dict_step_to_reachable_set.keys())[delta_steps:step + 1]:
                list_nodes_grand_parent = self.dict_step_to_reachable_set[step - delta_steps]
                list_nodes_grand_parent_propagated = self._propagate_reachable_set(list_nodes_grand_parent,
                                                                                   steps=delta_steps)
                cc_nodes, cc_obj_2_node = get_cc_at_time(step)
                cc_grandparents_prop, cc_obj_2_grandparent_node_prop = create_cc(list_nodes_grand_parent_propagated,
                                                                                 list_nodes_grand_parent)
                for rect_gp_prop, node_grand_parent in cc_obj_2_grandparent_node_prop.items():
                    colliding_rects = cc_nodes.find_all_colliding_objects(rect_gp_prop)
                    for rect in colliding_rects:
                        node = cc_obj_2_node[rect]
                        node.add_grandparent_node(node_grand_parent)
                        node_grand_parent.add_grandchild_node(node)

    def _save_to_pickle(self):
        """Saves computation result as a pickle file."""
        os.makedirs(self.config.general.path_offline_data, exist_ok=True)

        dict_data = self._extract_information()

        size_grid = self.config.reachable_set.size_grid

        if self.config.planning.coordinate_system == "CART":
            a_v_str = f"amax{self.config.vehicle.ego.a_max}_" \
                      f"vmax{self.config.vehicle.ego.v_max}_"
        else:
            a_v_str = f"alonmax{self.config.vehicle.ego.a_lon_max}_" \
                      f"alatmax{self.config.vehicle.ego.a_lat_max}_" \
                      f"vlonmax{self.config.vehicle.ego.v_lon_max}_" \
                      f"vlatmax{self.config.vehicle.ego.v_lat_max}_"

        self.config.reachable_set.name_pickle_offline = \
            f"offline_nt{self.max_evaluated_step}_" \
            f"{self.config.planning.coordinate_system}_" \
            + a_v_str + \
            f"ms{self.config.reachable_set.n_multi_steps}_" \
            f"dx{size_grid}_" \
            f"ver{dict_data['__version__']}" \
            f".pickle"

        with open(self.path_offline_file, 'wb') as f:
            pickle.dump(dict_data, f)

        util_logger.print_and_log_info(logger, f"Computation result saved to pickle file: {self.path_offline_file}")

    def create_projection_matrices(self):
        size_grid = self.config.reachable_set.size_grid
        size_grid_div = 1 / size_grid
        for t, base_set_list in self.dict_step_to_drivable_area.items():
            assert (len(self.dict_step_to_drivable_area) == len(self.dict_step_to_reachable_set))
            lon_min = size_grid * math.floor(min([x.p_lon_center for x in base_set_list]) * size_grid_div)
            lon_max = size_grid * math.ceil(max([x.p_lon_center for x in base_set_list]) * size_grid_div)
            lat_min = size_grid * math.floor(min([x.p_lat_center for x in base_set_list]) * size_grid_div)
            lat_max = size_grid * math.ceil(max([x.p_lat_center for x in base_set_list]) * size_grid_div)
            n_lon = round((lon_max - lon_min) * size_grid_div)
            n_lat = round((lat_max - lat_min) * size_grid_div)
            n_lon = max(1, n_lon)
            n_lat = max(1, n_lat)
            lon_orig_index = round(-lon_min * size_grid_div)
            lat_orig_index = round(-lat_min * size_grid_div)

            def position_to_grid_index2d(pos_lon: float, pos_lat: float) -> Tuple[int, int]:
                return (lon_orig_index + math.floor(pos_lon * size_grid_div),
                        lat_orig_index + math.floor(pos_lat * size_grid_div))

            def position_to_grid_index1d(pos_lon: float, pos_lat: float) -> int:
                lon, lat = position_to_grid_index2d(pos_lon, pos_lat)
                return lon * n_lat + lat

            # organize into grid
            grid_index_to_reachset = defaultdict(list)
            for index_reachset, reach_set in enumerate(self.dict_step_to_reachable_set[t]):
                grid_index_to_reachset[position_to_grid_index1d(reach_set.position_rectangle.p_lon_center,
                                                                reach_set.position_rectangle.p_lat_center)].append(
                    index_reachset)

            for reach_lists in grid_index_to_reachset.values():
                assert len(reach_lists) == 1, f"more than 1 reachset assigned to a cell!"

            coordinates = list(grid_index_to_reachset.keys())
            coordinates.sort()
            projection_matrix = np.zeros(shape=[len(self.dict_step_to_reachable_set[t]), n_lon * n_lat], dtype=bool)
            for coordinate in coordinates:
                projection_matrix[grid_index_to_reachset[coordinate], coordinate] = True

        raise

    def _extract_information(self):
        """Extracts essential information from the computation result."""
        dict_time_to_list_tuples_reach_node_attributes = defaultdict(list)
        dict_time_to_adjacency_matrices_parent = dict()
        dict_time_to_adjacency_matrices_grandparent = defaultdict(dict)
        reachset_bb_ll = dict()
        reachset_bb_ur = dict()
        size_grid = self.config.reachable_set.size_grid
        size_grid_div = 1 / size_grid
        for time_step, list_nodes in self.dict_step_to_reachable_set.items():
            lon_min = size_grid * math.floor(
                min([x.position_rectangle.p_lon_center for x in list_nodes]) * size_grid_div)
            lon_max = size_grid * math.ceil(
                max([x.position_rectangle.p_lon_center for x in list_nodes]) * size_grid_div)
            lat_min = size_grid * math.floor(
                min([x.position_rectangle.p_lat_center for x in list_nodes]) * size_grid_div)
            lat_max = size_grid * math.ceil(
                max([x.position_rectangle.p_lat_center for x in list_nodes]) * size_grid_div)
            if lon_min == 0.0:
                lon_min = size_grid * math.floor(
                    min([x.position_rectangle.p_lon_min for x in list_nodes]) * size_grid_div)
                lon_max = size_grid * math.ceil(
                    max([x.position_rectangle.p_lon_max for x in list_nodes]) * size_grid_div)
            if lat_min == 0.0:
                lat_min = size_grid * math.floor(
                    min([x.position_rectangle.p_lat_min for x in list_nodes]) * size_grid_div)
                lat_max = size_grid * math.ceil(
                    max([x.position_rectangle.p_lat_max for x in list_nodes]) * size_grid_div)
            reachset_bb_ll[time_step] = np.array([lon_min, lat_min])
            reachset_bb_ur[time_step] = np.array([lon_max, lat_max])

            for node in list_nodes:
                tuple_attribute = (
                    round(node.p_lon_min, 6), round(node.p_lat_min, 6), round(node.p_lon_max, 6),
                    round(node.p_lat_max, 6),
                    round(node.v_lon_min, 6), round(node.v_lat_min, 6), round(node.v_lon_max, 6),
                    round(node.v_lat_max, 6))
                dict_time_to_list_tuples_reach_node_attributes[time_step].append(tuple_attribute)

            # parent-child relationship
            if time_step >= 1:
                list_nodes_parent = self.dict_step_to_reachable_set[time_step - 1]

                matrix_adjacency_tmp = list()
                for node in list_nodes:
                    list_adjacency = [(node_parent in node.list_nodes_parent) for node_parent in list_nodes_parent]
                    matrix_adjacency_tmp.append(list_adjacency)

                matrix_adjacency_dense = np.array(matrix_adjacency_tmp, dtype=bool)
                matrix_adjacency_sparse = sparse.csr_matrix(matrix_adjacency_dense, dtype=bool)
                dict_time_to_adjacency_matrices_parent[time_step] = matrix_adjacency_sparse

            # grandparent-grandchild relationship
            for delta_steps in range(2, self.config.reachable_set.n_multi_steps):
                if time_step - delta_steps not in self.dict_step_to_reachable_set:
                    break
                list_nodes_grandparent = self.dict_step_to_reachable_set[time_step - delta_steps]
                matrix_adjacency_tmp = list()
                for node in list_nodes:
                    list_adjacency = [(node_grandparent in node.dict_time_to_set_nodes_grandparent[delta_steps])
                                      for node_grandparent in list_nodes_grandparent]
                    matrix_adjacency_tmp.append(list_adjacency)

                matrix_adjacency_sparse = sparse.csr_matrix(np.array(matrix_adjacency_tmp, dtype=bool), dtype=bool)
                dict_time_to_adjacency_matrices_grandparent[time_step][delta_steps] = matrix_adjacency_sparse

        dict_data = dict()
        dict_data["node_attributes"] = dict_time_to_list_tuples_reach_node_attributes
        dict_data["adjacency_matrices_parent"] = dict_time_to_adjacency_matrices_parent
        dict_data["adjacency_matrices_grandparent"] = dict(dict_time_to_adjacency_matrices_grandparent)
        dict_data["reachset_bb_ll"] = reachset_bb_ll
        dict_data["reachset_bb_ur"] = reachset_bb_ur
        dict_data["config.vehicle"] = self.config.vehicle
        dict_data["config.reachable_set"] = self.config.reachable_set
        dict_data["coordinate_system"] = self.config.planning.coordinate_system
        dict_data["__version__"] = __version__
        return dict_data

    def _prune_nodes_not_reaching_final_step(self):
        raise NotImplementedError
