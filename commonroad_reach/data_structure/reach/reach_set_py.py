import logging

logger = logging.getLogger(__name__)
from collections import defaultdict
from typing import List

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_analysis import ReachabilityAnalysis
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


class PyReachableSet:
    """Reachable set computation with Python backend."""

    def __init__(self, config: Configuration):
        self.config = config
        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        self._reachability_analysis = ReachabilityAnalysis(config)
        self._dict_time_to_base_set_propagated = defaultdict(list)
        self._dict_time_to_drivable_area = defaultdict(list)
        self._dict_time_to_reachable_set = defaultdict(list)
        self._dict_time_to_drivable_area[self.time_step_start] = self._reachability_analysis.initial_drivable_area
        self._dict_time_to_reachable_set[self.time_step_start] = self._reachability_analysis.initial_reachable_set
        self._prune_reachable_set = config.reachable_set.prune_nodes_not_reaching_final_time_step
        self._pruned = False
        self._list_time_steps_computed = [0]

    def drivable_area_at_time_step(self, time_step: int) -> List[ReachPolygon]:
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for drivable area retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self._dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int) -> List[ReachNode]:
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for reachable set retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self._dict_time_to_reachable_set[time_step]

    def compute_reachable_sets(self, time_step_start: int, time_step_end: int):
        """Calls reachable set computation functions for the specified time steps."""
        for time_step in range(time_step_start, time_step_end + 1):
            logger.debug(f"Computing reachable set for time step {time_step}")
            self._compute_at_time_step(time_step)
            self._list_time_steps_computed.append(time_step)

        if self.config.reachable_set.prune_nodes_not_reaching_final_time_step:
            self._prune_nodes_not_reaching_final_time_step()

    def _compute_at_time_step(self, time_step: int):
        self._compute_drivable_area_at_time_step(time_step)
        self._compute_reachable_set_at_time_step(time_step)

    def _compute_drivable_area_at_time_step(self, time_step: int):
        reachable_set_previous = self._dict_time_to_reachable_set[time_step - 1]
        drivable_area, base_set_propagated = \
            self._reachability_analysis.compute_drivable_area_at_time_step(time_step, reachable_set_previous)

        self._dict_time_to_drivable_area[time_step] = drivable_area
        self._dict_time_to_base_set_propagated[time_step] = base_set_propagated

    def _compute_reachable_set_at_time_step(self, time_step):
        base_sets_propagated = self._dict_time_to_base_set_propagated[time_step]
        drivable_area = self._dict_time_to_drivable_area[time_step]
        reachable_set = self._reachability_analysis.compute_reachable_set_at_time_step(time_step,
                                                                                       base_sets_propagated,
                                                                                       drivable_area)
        self._dict_time_to_reachable_set[time_step] = reachable_set

    def _prune_nodes_not_reaching_final_time_step(self):
        """Prunes nodes that don't reach the final time step.

        Iterates through reachability graph backward in time, discards nodes that don't have a child node.
        """
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
            self._dict_time_to_drivable_area[time_step] = [node.position_rectangle
                                                           for idx_node, node in enumerate(list_nodes)
                                                           if idx_node not in list_idx_nodes_to_be_deleted]
            self._dict_time_to_reachable_set[time_step] = [node for idx_node, node in enumerate(list_nodes)
                                                           if idx_node not in list_idx_nodes_to_be_deleted]

            cnt_nodes_after_pruning += len(list_nodes)

        self._pruned = True

        message = f"\t#Nodes before pruning: \t{cnt_nodes_before_pruning}"
        print(message)
        logger.info(message)

        message = f"\t#Nodes after pruning: \t{cnt_nodes_after_pruning}"
        print(message)
        logger.info(message)
