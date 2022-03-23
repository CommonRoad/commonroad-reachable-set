import logging
from typing import List

from commonroad_reach.data_structure.collision_checker_cpp import CppCollisionChecker
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.data_structure.reach.reach_set import ReachableSet
from commonroad_reach.utility import reach_operation

logger = logging.getLogger(__name__)

from commonroad_reach.data_structure.configuration import Configuration


class PyReachableSet(ReachableSet):
    """Reachable set computation with Python backend."""

    def __init__(self, config: Configuration):
        super().__init__(config)
        self.dict_time_to_drivable_area[self.time_step_start] = self._construct_initial_drivable_area()
        self.dict_time_to_reachable_set[self.time_step_start] = self._construct_initial_reachable_set()
        self._initialize_zero_state_polygons()
        self.collision_checker = CppCollisionChecker(self.config)

        logger.info("PyReachableSet initialized.")

    def _construct_initial_drivable_area(self) -> List[ReachPolygon]:
        tuple_vertices = reach_operation.generate_tuple_vertices_position_rectangle_initial(self.config)

        return [ReachPolygon.from_rectangle_vertices(*tuple_vertices)]

    def _construct_initial_reachable_set(self) -> List[ReachNode]:
        tuple_vertices_polygon_lon, tuple_vertices_polygon_lat = \
            reach_operation.generate_tuples_vertices_polygons_initial(self.config)

        polygon_lon = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lon)
        polygon_lat = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lat)

        return [ReachNode(polygon_lon, polygon_lat, self.config.planning.time_step_start)]

    def _initialize_zero_state_polygons(self):
        """Initializes the zero-state polygons of the system.

        Computation of the reachable set of an LTI system requires the zero-state response of the system.
        """
        self.polygon_zero_state_lon = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                self.config.vehicle.ego.a_lon_min,
                                                                                self.config.vehicle.ego.a_lon_max)

        self.polygon_zero_state_lat = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                self.config.vehicle.ego.a_lat_min,
                                                                                self.config.vehicle.ego.a_lat_max)

    def compute(self, time_step_start: int, time_step_end: int):
        for time_step in range(time_step_start, time_step_end + 1):
            logger.debug(f"Computing reachable set for time step {time_step}")
            self._compute_drivable_area_at_time_step(time_step)
            self._compute_reachable_set_at_time_step(time_step)
            self._list_time_steps_computed.append(time_step)

        if self.config.reachable_set.prune_nodes_not_reaching_final_time_step:
            self._prune_nodes_not_reaching_final_time_step()

    def _compute_drivable_area_at_time_step(self, time_step: int):
        """Computes the drivable area for the given time step.

        Steps:
            1. Propagate each node of the reachable set from the previous time step, resulting in propagated base sets.
            2. Project the base sets onto the position domain to obtain position rectangles.
            3. Merge and repartition these rectangles to potentially reduce the number of rectangles.
            4. Check for collisions with obstacles and road boundaries, and obtain collision-free rectangles.
            5. Merge and repartition the collision-free rectangles again to potentially reduce the number of rectangles.
        """
        reachable_set_previous = self.dict_time_to_reachable_set[time_step - 1]

        if len(reachable_set_previous) < 1:
            return None

        list_base_sets_propagated = self._propagate_reachable_set(reachable_set_previous)

        list_rectangles_projected = reach_operation.project_base_sets_to_position_domain(list_base_sets_propagated)

        # repartition, then collision check
        if self.config.reachable_set.mode_repartition == 1:
            list_rectangles_repartitioned = \
                reach_operation.create_repartitioned_rectangles(list_rectangles_projected,
                                                                self.config.reachable_set.size_grid)

            drivable_area = \
                reach_operation.check_collision_and_split_rectangles(self.collision_checker, time_step,
                                                                     list_rectangles_repartitioned,
                                                                     self.config.reachable_set.radius_terminal_split)

        # collision check, then repartition
        elif self.config.reachable_set.mode_repartition == 2:
            list_rectangles_collision_free = \
                reach_operation.check_collision_and_split_rectangles(self.collision_checker, time_step,
                                                                     list_rectangles_projected,
                                                                     self.config.reachable_set.radius_terminal_split)
            drivable_area = reach_operation.create_repartitioned_rectangles(list_rectangles_collision_free,
                                                                            self.config.reachable_set.size_grid_2nd)

        # repartition, collision check, then repartition again
        elif self.config.reachable_set.mode_repartition == 3:
            list_rectangles_repartitioned = \
                reach_operation.create_repartitioned_rectangles(list_rectangles_projected,
                                                                self.config.reachable_set.size_grid)

            list_rectangles_collision_free = \
                reach_operation.check_collision_and_split_rectangles(self.collision_checker, time_step,
                                                                     list_rectangles_repartitioned,
                                                                     self.config.reachable_set.radius_terminal_split)

            drivable_area = reach_operation.create_repartitioned_rectangles(list_rectangles_collision_free,
                                                                            self.config.reachable_set.size_grid_2nd)

        else:
            raise Exception("Invalid mode for repartition.")

        self.dict_time_to_drivable_area[time_step] = drivable_area
        self.dict_time_to_base_set_propagated[time_step] = list_base_sets_propagated

    def _compute_reachable_set_at_time_step(self, time_step):
        """Computes the reachable set for the given time step.

        Steps:
            1. construct reach nodes from drivable area and the propagated based sets.
            2. update parent-child relationship of the nodes.
        """
        base_sets_propagated = self.dict_time_to_base_set_propagated[time_step]
        drivable_area = self.dict_time_to_drivable_area[time_step]

        if not drivable_area:
            return None

        list_nodes = reach_operation.construct_reach_nodes(drivable_area, base_sets_propagated)

        reachable_set = reach_operation.connect_children_to_parents(time_step, list_nodes)

        self.dict_time_to_reachable_set[time_step] = reachable_set

    def _propagate_reachable_set(self, list_nodes: List[ReachNode]) -> List[ReachNode]:
        """Propagates the nodes of the reachable set."""
        list_base_sets_propagated = []

        for node in list_nodes:
            try:
                polygon_lon_propagated = reach_operation.propagate_polygon(node.polygon_lon,
                                                                           self.polygon_zero_state_lon,
                                                                           self.config.planning.dt,
                                                                           self.config.vehicle.ego.v_lon_min,
                                                                           self.config.vehicle.ego.v_lon_max)

                polygon_lat_propagated = reach_operation.propagate_polygon(node.polygon_lat,
                                                                           self.polygon_zero_state_lat,
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

    def _prune_nodes_not_reaching_final_time_step(self):
        logger.info("Pruning nodes not reaching final time step.")
        cnt_nodes_before_pruning = cnt_nodes_after_pruning = len(self.reachable_set_at_time_step(self.time_step_end))

        for time_step in range(self.time_step_end - 1, self.time_step_start - 1, -1):
            list_nodes = self.reachable_set_at_time_step(time_step)
            cnt_nodes_before_pruning += len(list_nodes)

            list_idx_nodes_to_be_deleted = list()
            for idx_node, node in enumerate(list_nodes):
                # discard the node if it has no child node
                if not node.list_nodes_child:
                    list_idx_nodes_to_be_deleted.append(idx_node)
                    # iterate through its parent nodes and disconnect them
                    for node_parent in node.list_nodes_parent:
                        node_parent.remove_child_node(node)

            # update drivable area and reachable set dictionaries
            self.dict_time_to_drivable_area[time_step] = [node.position_rectangle
                                                          for idx_node, node in enumerate(list_nodes)
                                                          if idx_node not in list_idx_nodes_to_be_deleted]
            self.dict_time_to_reachable_set[time_step] = [node for idx_node, node in enumerate(list_nodes)
                                                          if idx_node not in list_idx_nodes_to_be_deleted]

            cnt_nodes_after_pruning += len(self.dict_time_to_reachable_set[time_step])

        self._pruned = True

        message = f"\t#Nodes before pruning: \t{cnt_nodes_before_pruning}"
        print(message)
        logger.info(message)

        message = f"\t#Nodes after pruning: \t{cnt_nodes_after_pruning}"
        print(message)
        logger.info(message)
