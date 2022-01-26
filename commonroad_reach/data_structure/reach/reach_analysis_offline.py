import logging

logger = logging.getLogger(__name__)
from typing import List, Tuple, Dict

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import reach_operation


class OfflineReachabilityAnalysis:
    def __init__(self, config: Configuration):
        self.config = config
        self.polygon_zero_state_lon = None
        self.polygon_zero_state_lat = None

        if not self.config.planning.coordinate_system == "CART":
            message = "Offline reachable set computation only supports Cartesian coordinate system."
            logger.error(message)
            raise Exception(message)

        self._initialize_zero_state_polygons()

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

    def compute_drivable_area_at_time_step(self, time_step: int,
                                           reachable_set_previous: List[ReachNodeMultiGeneration]) \
            -> Tuple[List[ReachPolygon], List[ReachNodeMultiGeneration]]:
        """Computes the drivable area for the given time step.

        Steps:
            1. Propagate each node of the reachable set from the previous time step, resulting in propagated base sets.
            2. Project the base sets onto the position domain to obtain the position rectangles.
            3. Merge and repartition these rectangles to potentially reduce the number of rectangles.
            4. Adapt position rectangles to Cartesian grid.
            5. Remove rectangles out of Kamm's friction circle.
        """
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

        return drivable_area, list_base_sets_propagated

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

    @staticmethod
    def compute_reachable_set_at_time_step(time_step: int, base_set_propagated, drivable_area) \
            -> List[ReachNodeMultiGeneration]:
        """Computes the reachable set for the given time step.

        Steps:
            1. create a list of base sets adapted to the drivable area.
            2. create a list of reach nodes from the list of adapted base sets.
        """
        if not drivable_area:
            return []

        list_base_sets_adapted = reach_operation.adapt_base_sets_to_drivable_area(drivable_area,
                                                                                  base_set_propagated,
                                                                                  has_multi_generation=True)

        reachable_set_time_step_current = reach_operation.create_nodes_of_reachable_set(time_step,
                                                                                        list_base_sets_adapted)

        return reachable_set_time_step_current

    def determine_grandparent_relationship(self, dict_time_to_reachable_set: Dict[int, List[ReachNodeMultiGeneration]]):
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
