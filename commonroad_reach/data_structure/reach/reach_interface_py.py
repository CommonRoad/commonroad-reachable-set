from collections import defaultdict
from typing import List

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_analysis import ReachabilityAnalysis
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


class PyReachableSetInterface:
    """Interface to work with reachable sets with python backend."""

    def __init__(self, config: Configuration):
        self.config = config
        self._reachability_analysis = ReachabilityAnalysis(config)
        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        self.dict_time_to_base_set_propagated = dict()
        self.dict_time_to_drivable_area = dict()
        self.dict_time_to_reachable_set = dict()
        self.dict_time_to_drivable_area_pruned = defaultdict(list)
        self.dict_time_to_reachable_set_pruned = defaultdict(list)

        self._initialize()

    def _initialize(self):
        """Initializes drivable area and reachable set of the initial time step."""
        self.dict_time_to_drivable_area[self.time_step_start] = self._reachability_analysis.initial_drivable_area
        self.dict_time_to_reachable_set[self.time_step_start] = self._reachability_analysis.initial_reachable_set

    @property
    def reachability_analysis(self):
        return self._reachability_analysis

    def drivable_area_at_time_step(self, time_step: int) -> List[ReachPolygon]:
        assert time_step in self.dict_time_to_drivable_area, "<PyReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int) -> List[ReachNode]:
        assert time_step in self.dict_time_to_drivable_area, "<PyReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_reachable_set[time_step]

    def pruned_reachable_set_at_time_step(self, time_step: int) -> List[ReachNode]:
        assert time_step in self.dict_time_to_drivable_area, "<PyReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_reachable_set_pruned[time_step]

    def compute_reachable_sets(self, time_step_start: int = 1, time_step_end: int = 0):
        """Computes reachable sets for the specified time steps."""
        assert time_step_start != 0, "<PyReachableSetInterface> Time step should not start with 0."

        if not time_step_end:
            time_step_end = self.time_step_end

        for time_step in range(time_step_start, time_step_end + 1):
            self._compute_at_time_step(time_step)

        if self.config.reachable_set.prune_nodes_not_reaching_final_time_step:
            self._prune_reachable_set()

    def _compute_at_time_step(self, time_step: int):
        """Computes drivable area and reachable set of the time step."""
        self._compute_drivable_area_at_time_step(time_step)
        self._compute_reachable_set_at_time_step(time_step)

    def _compute_drivable_area_at_time_step(self, time_step: int):
        """Computes drivable area of the time step.

        Drivable area is computed based on reachable set of the last time step.
        """
        reachable_set_previous = self.dict_time_to_reachable_set[time_step - 1]
        drivable_area, base_set_propagated = \
            self._reachability_analysis.compute_drivable_area_at_time_step(time_step, reachable_set_previous)

        self.dict_time_to_drivable_area[time_step] = drivable_area
        self.dict_time_to_base_set_propagated[time_step] = base_set_propagated

    def _compute_reachable_set_at_time_step(self, time_step):
        """Compute reachable set of the time step.

        Reachable set is computed based on the drivable area and the list of propagated nodes. It is also based on the
        reachable set of the last time step.
        """
        base_sets_propagated = self.dict_time_to_base_set_propagated[time_step]
        drivable_area = self.dict_time_to_drivable_area[time_step]

        reachable_set = self._reachability_analysis. \
            compute_reachable_set_at_time_step(time_step, base_sets_propagated, drivable_area)

        self.dict_time_to_reachable_set[time_step] = reachable_set

    def _prune_reachable_set(self):
        """Prunes nodes not reaching the final time step.

        Iterates through reachable sets in backward direction and discard nodes not reaching the final time step.
        """
        cnt_nodes_before_pruning = 0
        cnt_nodes_after_pruning = 0

        cnt_nodes_before_pruning += len(self.reachable_set_at_time_step(self.time_step_end))

        for time_step in range(self.time_step_end - 1, self.time_step_start - 1, -1):
            list_nodes = self.reachable_set_at_time_step(time_step)
            cnt_nodes_before_pruning += len(list_nodes)

            for node in list_nodes:
                # discard if the node has no child node
                if not node.list_nodes_child:
                    # iterate through parent nodes and disconnect them
                    for node_parent in node.list_nodes_parent:
                        node_parent.remove_child_node(node)
                    node.list_nodes_parent.clear()
                    list_nodes.remove(node)

            cnt_nodes_after_pruning += len(list_nodes)

        print(f"\t#nodes before pruning: \t{cnt_nodes_before_pruning}")
        print(f"\t#nodes after pruning: \t{cnt_nodes_after_pruning}")
