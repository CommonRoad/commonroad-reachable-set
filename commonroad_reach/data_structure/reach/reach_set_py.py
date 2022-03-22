import logging
from typing import List

from commonroad_reach.data_structure.collision_checker_py import PyCollisionChecker
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
        self.polygon_zero_state_lon = None
        self.polygon_zero_state_lat = None
        self._collision_checker = None
        self._initialize_zero_state_polygons()
        self._initialize_collision_checker()
        self.dict_time_to_drivable_area[self.time_step_start] = self.construct_initial_drivable_area()
        self.dict_time_to_reachable_set[self.time_step_start] = self.construct_initial_reachable_set()

    def _initialize_zero_state_polygons(self):
        """Initializes the zero-state polygons of the system.

        Computation of the reachable set of an LTI system requires the zero-state response and the zero-input response
        of the system.
        """
        self.polygon_zero_state_lon = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                self.config.vehicle.ego.a_lon_min,
                                                                                self.config.vehicle.ego.a_lon_max)

        self.polygon_zero_state_lat = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                self.config.vehicle.ego.a_lat_min,
                                                                                self.config.vehicle.ego.a_lat_max)

    def _initialize_collision_checker(self):
        """Initializes collision checker."""
        mode = self.config.reachable_set.mode
        if mode == 1:
            self._collision_checker = PyCollisionChecker(self.config)

        elif mode == 2:
            try:
                from commonroad_reach.data_structure.collision_checker_cpp import CppCollisionChecker

            except ImportError:
                message = "Importing C++ collision checker failed."
                logger.exception(message)
                print(message)

            else:
                self._collision_checker = CppCollisionChecker(self.config)

        else:
            message = "Specified mode ID is invalid."
            logger.error(message)
            raise Exception(message)

    def construct_initial_drivable_area(self) -> List[ReachPolygon]:
        """Drivable area at the initial time step.

        Constructed directly from the config file.
        """
        tuple_vertices = reach_operation.generate_tuple_vertices_position_rectangle_initial(self.config)

        return [ReachPolygon.from_rectangle_vertices(*tuple_vertices)]

    def construct_initial_reachable_set(self) -> List[ReachNode]:
        """Reachable set at the initial time step.

        Vertices of the polygons are constructed directly from the config file.
        """
        tuple_vertices_polygon_lon, tuple_vertices_polygon_lat = \
            reach_operation.generate_tuples_vertices_polygons_initial(self.config)

        polygon_lon = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lon)
        polygon_lat = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lat)

        return [ReachNode(polygon_lon, polygon_lat, self.config.planning.time_step_start)]

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
            2. Project the base sets onto the position domain to obtain the position rectangles.
            3. Merge and repartition these rectangles to potentially reduce the number of rectangles.
            4. Check for collisions with obstacles and road boundaries, and obtain collision-free rectangles.
            5. Merge and repartition the collision-free rectangles again to potentially reduce the number of rectangles.
        """
        reachable_set_previous = self.dict_time_to_reachable_set[time_step - 1]

        if len(reachable_set_previous) < 1:
            return None

        list_base_sets_propagated = self._propagate_reachable_set(reachable_set_previous)

        list_rectangles_projected = reach_operation.project_base_sets_to_position_domain(list_base_sets_propagated)

        list_rectangles_repartitioned = \
            reach_operation.create_repartitioned_rectangles(list_rectangles_projected,
                                                            self.config.reachable_set.size_grid)

        list_rectangles_collision_free = \
            reach_operation.check_collision_and_split_rectangles(self._collision_checker, time_step,
                                                                 list_rectangles_repartitioned,
                                                                 self.config.reachable_set.radius_terminal_split)

        drivable_area = reach_operation.create_repartitioned_rectangles(list_rectangles_collision_free,
                                                                        self.config.reachable_set.size_grid_2nd)

        self.dict_time_to_drivable_area[time_step] = drivable_area
        self.dict_time_to_base_set_propagated[time_step] = list_base_sets_propagated

    def _compute_reachable_set_at_time_step(self, time_step):
        """Computes the reachable set for the given time step.

        Steps:
            1. create a list of base sets adapted to the drivable area.
            2. create a list of reach nodes from the list of adapted base sets.
        """
        base_sets_propagated = self.dict_time_to_base_set_propagated[time_step]
        drivable_area = self.dict_time_to_drivable_area[time_step]

        if not drivable_area:
            return None

        list_base_sets_adapted = reach_operation.adapt_base_sets_to_drivable_area(drivable_area, base_sets_propagated)

        reachable_set_current_step = reach_operation.create_nodes_of_reachable_set(time_step, list_base_sets_adapted)

        self.dict_time_to_reachable_set[time_step] = reachable_set_current_step

    def _propagate_reachable_set(self, list_nodes: List[ReachNode]) -> List[ReachNode]:
        """Propagates the nodes of the reachable set from the previous time step."""
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
                if not node.nodes_child:
                    list_idx_nodes_to_be_deleted.append(idx_node)
                    # iterate through its parent nodes and disconnect them
                    for node_parent in node.nodes_parent:
                        node_parent.remove_child_node(node)

            # update drivable area and reachable set dictionaries
            self.dict_time_to_drivable_area[time_step] = [node.position_rectangle
                                                          for idx_node, node in enumerate(list_nodes)
                                                          if idx_node not in list_idx_nodes_to_be_deleted]
            self.dict_time_to_reachable_set[time_step] = [node for idx_node, node in enumerate(list_nodes)
                                                          if idx_node not in list_idx_nodes_to_be_deleted]

            cnt_nodes_after_pruning += len(list_nodes)

        self._pruned = True

        message = f"\t#Nodes before pruning: \t{cnt_nodes_before_pruning}"
        print(message)
        logger.info(message)

        message = f"\t#Nodes after pruning: \t{cnt_nodes_after_pruning}"
        print(message)
        logger.info(message)
