from typing import List, Tuple

from commonroad_reachset.data_structure.configuration import Configuration
from commonroad_reachset.data_structure.reach.reach_node import ReachNode
from commonroad_reachset.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reachset.data_structure.collision_checker import CollisionChecker
from commonroad_reachset.utility import reach_operation


class ReachabilityAnalysis:
    def __init__(self, config: Configuration, collision_checker: CollisionChecker = None):
        self._config = config
        self._collision_checker = collision_checker or CollisionChecker(config)
        self._initialize_zero_state_polygons()

    def _initialize_zero_state_polygons(self):
        """Initializes the zero-state polygons of the system.

        Computation of the reachable set of an LTI system requires the zero-state response
        and the zero-input response of the system.
        """
        self._polygon_zero_state_lon = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                    self.config.vehicle.ego.a_lon_min,
                                                                                    self.config.vehicle.ego.a_lon_max)

        self._polygon_zero_state_lat = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                    self.config.vehicle.ego.a_lat_min,
                                                                                    self.config.vehicle.ego.a_lat_max)

    @property
    def config(self) -> Configuration:
        return self._config

    @property
    def collision_checker(self) -> CollisionChecker:
        return self._collision_checker

    @property
    def polygon_zero_state_lon(self) -> ReachPolygon:
        return self._polygon_zero_state_lon

    @property
    def polygon_zero_state_lat(self) -> ReachPolygon:
        return self._polygon_zero_state_lat

    @property
    def initial_drivable_area(self) -> List[ReachPolygon]:
        """Drivable area at the initial time step.

        Constructed directly from the config file.
        """
        tuple_vertices = reach_operation.generate_tuple_vertices_position_rectangle_initial(self.config)

        return [ReachPolygon.from_rectangle_vertices(*tuple_vertices)]

    @property
    def initial_reachable_set(self) -> List[ReachNode]:
        """Reachable set at the initial time step.

        Vertices of the longitudinal and lateral polygons are constructed directly from the config file.
        """
        tuple_vertices_polygon_lon, tuple_vertices_polygon_lat = \
            reach_operation.generate_tuples_vertices_polygons_initial(self.config)

        polygon_lon = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lon)
        polygon_lat = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lat)

        return [ReachNode(polygon_lon, polygon_lat, self._config.planning.time_step_start)]

    def compute_drivable_area_at_time_step(self, time_step: int, reachable_set_previous: List[ReachNode]) \
            -> Tuple[List[ReachPolygon], List[ReachNode]]:
        """Computes the drivable area at the specified time step.

        Steps:
            1. Propagate each node of the reachable set from the last time step.
               This forms a list of propagated base sets.
            2. Project the base sets onto the position domain.
            3. Merge and repartition these rectangles to reduce computation load.
            4. Check for collision and split the repartitioned rectangles into
               collision-free rectangles.
            5. Merge and repartition the collision-free rectangles again to
               reduce computation load. This is the final drivable area.
        """
        if len(reachable_set_previous) < 1:
            return [], []

        # Step 1
        list_base_sets_propagated = self._propagate_reachable_set(reachable_set_previous)

        # Step 2
        list_rectangles_projected = reach_operation.project_base_sets_to_position_domain(list_base_sets_propagated)

        # Step 3
        list_rectangles_repartitioned = reach_operation.create_repartitioned_rectangles(
            list_rectangles_projected, self.config.reachable_set.size_grid)

        # Step 4
        list_rectangles_collision_free = reach_operation.check_collision_and_split_rectangles(
            self.collision_checker, time_step, list_rectangles_repartitioned,
            self.config.reachable_set.radius_terminal_split)

        # Step 5
        drivable_area = reach_operation.create_repartitioned_rectangles(list_rectangles_collision_free,
                                                                           self.config.reachable_set.size_grid_2nd)

        return drivable_area, list_base_sets_propagated

    def _propagate_reachable_set(self, list_nodes: List[ReachNode]) -> List[ReachNode]:
        """Propagates the nodes of the reachable set from the last time step.

        Args:
            list_nodes (List[ReachNode]): reach nodes from the previous time step.
        """
        list_base_sets_propagated = []

        for node in list_nodes:
            try:
                polygon_lon_propagated = reach_operation.propagate_polygon(node.polygon_lon,
                                                                              self._polygon_zero_state_lon,
                                                                              self.config.planning.dt,
                                                                              self.config.vehicle.ego.v_lon_min,
                                                                              self.config.vehicle.ego.v_lon_max)

                polygon_lat_propagated = reach_operation.propagate_polygon(node.polygon_lat,
                                                                              self._polygon_zero_state_lat,
                                                                              self.config.planning.dt,
                                                                              self.config.vehicle.ego.v_lat_min,
                                                                              self.config.vehicle.ego.v_lat_max)
            except (ValueError, RuntimeError):
                pass

            else:
                base_set_propagated = ReachNode(polygon_lon_propagated, polygon_lat_propagated, node.time_step)
                base_set_propagated.source_propagation = node
                list_base_sets_propagated.append(base_set_propagated)

        return list_base_sets_propagated

    @staticmethod
    def compute_reachable_set_at_time_step(time_step: int, base_set_propagated, drivable_area) -> List[ReachNode]:
        """Computes the reachable set at the specified time step.

        Steps:
            1. create a list of new base sets cut down with rectangles of
               the drivable area.
            2. create the list of nodes for the final reachable set.
        """
        if not drivable_area:
            return []

        # Step 1
        list_base_sets_adapted = reach_operation.adapt_base_sets_to_drivable_area(drivable_area,
                                                                                     base_set_propagated)
        # Step 2
        reachable_set_time_step_current = reach_operation.create_nodes_of_reachable_set(time_step,
                                                                                           list_base_sets_adapted)

        return reachable_set_time_step_current
