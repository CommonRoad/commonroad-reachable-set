import logging
import math
import os
import pickle
import time
from typing import List, Dict, Tuple, Optional

from commonroad_reach.__version__ import __version__
import numpy as np
from scipy import sparse

from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.data_structure.reach.reach_set import ReachableSet

logger = logging.getLogger(__name__)

from collections import defaultdict

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.utility import reach_operation


class PyGraphReachableSetOffline(ReachableSet):
    """Offline step in the graph-based reachable set computation with Python backend."""

    def __init__(self, config: Configuration):
        self._dict_time_to_drivable_area: Dict[int, List[ReachPolygon]]
        self._dict_time_to_reachable_set: Dict[int, List[ReachNodeMultiGeneration]]
        super().__init__(config)

        if config.planning.coordinate_system != "CART":
            message = "Multi-step reachable set computation only supports Cartesian coordinate system."
            logger.error(message)
            raise Exception(message)

        self.polygon_zero_state_lon: Dict[int, ReachPolygon] = dict()
        self.polygon_zero_state_lat: Dict[int, ReachPolygon] = dict()
        self._initialize_zero_state_polygons()

        self._dict_time_to_drivable_area[self.time_step_start] = self.initial_drivable_area
        self._dict_time_to_reachable_set[self.time_step_start] = self.initial_reachable_set

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
            self.polygon_zero_state_lon[steps] = reach_operation.create_zero_state_polygon(dt,
                                                                                           -self.config.vehicle.ego.a_max,
                                                                                           self.config.vehicle.ego.a_max)

            self.polygon_zero_state_lat[steps] = reach_operation.create_zero_state_polygon(dt,
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
        for time_step in range(time_step_start + 1, time_step_end + 1):
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

            if self.config.reachable_set.n_multi_steps >= 2:
                self._determine_grandparent_relationship(time_step)

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

        # TODO: consider friction circle when regular grid is enforced in repartitioning
        # drivable_area = \
        #     reach_operation.remove_rectangles_out_of_kamms_circle(time_step * self.config.planning.dt,
        #                                                           self.config.vehicle.ego.a_max,
        #                                                           list_rectangles_adapted)

        self._dict_time_to_drivable_area[time_step] = list_rectangles_adapted
        self._dict_time_to_base_set_propagated[time_step] = list_base_sets_propagated

    def _propagate_reachable_set(self, list_nodes: List[ReachNodeMultiGeneration], steps=1) -> List[
        ReachNodeMultiGeneration]:
        """Propagates the nodes of the reachable set from the previous time step."""
        assert steps >= 1
        list_base_sets_propagated = []
        dt = self.config.planning.dt * steps
        for node in list_nodes:
            try:
                polygon_lon_propagated = reach_operation.propagate_polygon(node.polygon_lon,
                                                                           self.polygon_zero_state_lon[steps],
                                                                           dt,
                                                                           -self.config.vehicle.ego.v_max,
                                                                           self.config.vehicle.ego.v_max)

                polygon_lat_propagated = reach_operation.propagate_polygon(node.polygon_lat,
                                                                           self.polygon_zero_state_lat[steps],
                                                                           dt,
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

    def _determine_grandparent_relationship(self, time_step: int):
        """Determines grandparent-child relationship between nodes.

        The grandparent-child relationship is established if the grandparent can reach the grandchild by propagating
        two time steps.
        """
        logger.debug("Determining grandparent-grandchild relationship...")
        for delta_steps in range(2, self.config.reachable_set.n_multi_steps):
            for time_step in list(self.dict_time_to_reachable_set.keys())[delta_steps:time_step + 1]:
                list_nodes = self.dict_time_to_reachable_set[time_step]
                list_nodes_grand_parent = self.dict_time_to_reachable_set[time_step - delta_steps]
                list_nodes_grand_parent_propagated = self._propagate_reachable_set(list_nodes_grand_parent,
                                                                                   steps=delta_steps)

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
        os.makedirs(self.config.general.path_offline_data, exist_ok=True)

        dict_data = self._extract_information()

        size_grid = self.config.reachable_set.size_grid
        a_max = self.config.vehicle.ego.a_max
        v_max = self.config.vehicle.ego.v_max

        self.config.reachable_set.name_pickle_offline = \
            f"offline_{self.max_evaluated_time_step}_" \
            f"ms{self.config.reachable_set.n_multi_steps}_" \
            f"dx{size_grid}_" \
            f"amax{a_max}_" \
            f"vmax{v_max}_" \
            f"ver{dict_data['__version__']}" \
            f".pickle"
        f = open(self.path_offline_file, 'wb')
        pickle.dump(dict_data, f)
        f.close()

        print()
        message = f"Computation result saved to pickle file: {self.path_offline_file}"
        print(message)
        logger.info(message)
        # def _save_to_pickle(self):
        #     os.makedirs(self.config.general.path_offline_data, exist_ok=True)
        #     data = np.load(filename, allow_pickle=True)
        #     # dt = data['dt']
        #     matrices = []
        #     projections = []
        #     for t, adj_matrix in dict_time_to_adjacency_matrices_parent.items():
        #         tmp = csr_matrix((np.ones_like(data['dt0_' + 'E_indices' + str(t)]),
        #                           data['dt0_' + 'E_indices' + str(t)],
        #                           data['dt0_' + 'E_indptr' + str(t)]),
        #                          data['dt0_' + 'E_shape' + str(t)])
        #         matrices.append(tmp)
        #
        #         tmp = csr_matrix((np.ones_like(data['dt0_' + 'P_indices' + str(t)]),
        #                           data['dt0_' + 'P_indices' + str(t)],
        #                           data['dt0_' + 'P_indptr' + str(t)]),
        #                          data['dt0_' + 'P_shape' + str(t)])
        #         projections.append(tmp)

        # for t in range(data['nt']):
        #     for delta_timestep in range(dt):
        #         if 'data_gr' + str(t) + '_' + str(delta_timestep) in data:
        #             tmp = csr_matrix((data['data_gr' + str(t) + '_' + str(delta_timestep)],
        #                               data['indices_gr' + str(t) + '_' + str(delta_timestep)],
        #                               data['indptr_gr' + str(t) + '_' + str(delta_timestep)]),
        #                              data['shape_gr' + str(t) + '_' + str(delta_timestep)])
        #             matrices.append(tmp)

        # name_file = f"offline_{time_steps}_{size_grid}_{a_max}_{v_max}.pickle"
        #
        # name_file = f"offline_{time_steps}_{size_grid}_{a_max}_{v_max}.pickle"
        # f = open(f"{self.config.general.path_offline_data}{name_file}", 'wb')
        # pickle.dump(dict_data, f)
        # f.close()
        #
        # message = f"Computation result saved to pickle file: {name_file}"
        # print(message)
        # logger.info(message)

    def create_projection_matrices(self):
        size_grid = self.config.reachable_set.size_grid
        dx = size_grid / 2
        size_grid_div = 1 / size_grid
        for t, base_set_list in self._dict_time_to_drivable_area.items():
            assert (len(self.dict_time_to_drivable_area) == len(self.dict_time_to_reachable_set))
            l0 = min([x.p_lon_min for x in base_set_list])
            l1 = max([x.p_lon_max for x in base_set_list])
            l2 = min([x.p_lat_min for x in base_set_list])
            l3 = max([x.p_lat_max for x in base_set_list])

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
                print(lon, lat, lon * n_lat + lat, pos_lon, pos_lat)
                return lon * n_lat + lat

            # organize into grid
            grid_index_to_reachset = defaultdict(list)
            # iii=3
            # plt.figure()
            for index_reachset, reach_set in enumerate(self.dict_time_to_reachable_set[t]):
                # iii *= 0.95
                # v = np.array(reach_set.position_rectangle.vertices)
                # plt.fill(v[:, 0], v[:, 1], fill=False, lw=1.5 * iii, alpha=0.3)
                # continue
                grid_index_to_reachset[position_to_grid_index1d(reach_set.position_rectangle.p_lon_center,
                                                                reach_set.position_rectangle.p_lat_center)].append(
                    index_reachset)
                # continue
                # if len(grid_index_to_reachset[position_to_grid_index1d(reach_set.position_rectangle.p_lon_center, reach_set.position_rectangle.p_lat_center)])>1:
                #     plt.figure()
                #     x = np.arange(lon_min, lon_max+1e-6, size_grid)
                #     y = np.arange(lat_min, lat_max+1e-6, size_grid)
                #     vv = [(xx,yy) for xx in x for yy in y]
                #     vv = np.array(vv)
                #     plt.scatter(vv[:,0], vv[:,1])
                #     bounds = np.array([[lon_min,lat_min], [lon_min,lat_max],[lon_max,lat_max], [lon_max,lat_min]])
                #     plt.fill(bounds[:, 0], bounds[:, 1], fill=False)
                #     bounds = np.array([[l0, l2], [l0, l3], [l1, l3], [l1, l2]])
                #     plt.fill(bounds[:, 0], bounds[:, 1], fill=False)
                #     iii=6
                #     for inde in grid_index_to_reachset[position_to_grid_index1d(reach_set.position_rectangle.p_lon_center, reach_set.position_rectangle.p_lat_center)]:
                #         iii*=0.5
                #         v= np.array(self.dict_time_to_reachable_set[t][inde].position_rectangle.vertices)
                #         plt.fill(v[:,0], v[:,1], fill=False, lw=1.5*iii,alpha=0.3)
                #
                #     plt.title(f"{grid_index_to_reachset[position_to_grid_index1d(reach_set.position_rectangle.p_lon_center, reach_set.position_rectangle.p_lat_center)]}")
                #     plt.show(block=True)

            for reach_lists in grid_index_to_reachset.values():
                assert len(reach_lists) == 1, f"more than 1 reachset assigned to a cell!"

            coordinates = list(grid_index_to_reachset.keys())
            coordinates.sort()
            projection_matrix = np.zeros(shape=[len(self.dict_time_to_reachable_set[t]), n_lon * n_lat], dtype=bool)
            for coordinate in coordinates:
                # print(grid_index_to_reachset[coordinate])
                projection_matrix[grid_index_to_reachset[coordinate], coordinate] = True

            print("-----------------------------------")
            # print(t)
            print(projection_matrix)
        raise

    def _extract_information(self):
        """Extracts essential information from the computation result."""
        # self.create_projection_matrices()
        dict_time_to_list_tuples_reach_node_attributes = defaultdict(list)
        dict_time_to_adjacency_matrices_parent = dict()
        dict_time_to_adjacency_matrices_grandparent = defaultdict(dict)
        reachset_bb_ll = dict()
        reachset_bb_ur = dict()
        size_grid = self.config.reachable_set.size_grid
        size_grid_div = 1 / size_grid
        for time_step, list_nodes in self._dict_time_to_reachable_set.items():
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
                list_nodes_parent = self._dict_time_to_reachable_set[time_step - 1]

                matrix_adjacency_tmp = list()
                for node in list_nodes:
                    list_adjacency = [(node_parent in node.nodes_parent) for node_parent in list_nodes_parent]
                    matrix_adjacency_tmp.append(list_adjacency)

                matrix_adjacency_dense = np.array(matrix_adjacency_tmp, dtype=bool)
                matrix_adjacency_sparse = sparse.csr_matrix(matrix_adjacency_dense, dtype=bool)
                dict_time_to_adjacency_matrices_parent[time_step] = matrix_adjacency_sparse

            # grandparent-grandchild relationship
            for delta_steps in range(2, self.config.reachable_set.n_multi_steps):
                if time_step - delta_steps not in self._dict_time_to_reachable_set:
                    break
                list_nodes_grandparent = self._dict_time_to_reachable_set[time_step - delta_steps]
                matrix_adjacency_tmp = list()
                for node in list_nodes:
                    list_adjacency = [(node_grandparent in node.list_nodes_grandparent[delta_steps])
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
        dict_data["__version__"] = __version__
        return dict_data

    def _prune_nodes_not_reaching_final_time_step(self):
        pass
