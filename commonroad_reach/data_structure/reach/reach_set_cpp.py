from typing import List

import pycrreachset as reach

from commonroad_reach.data_structure.collision_checker_cpp import CppCollisionChecker
from commonroad_reach.data_structure.configuration import Configuration


class CppReachableSet:
    """Interface to work with C++ backend."""

    def __init__(self, config: Configuration):
        self.config = config
        config_cpp = config.convert_to_cpp_configuration()
        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        cc = CppCollisionChecker(config)
        self._reach = reach.ReachableSetInterface(config_cpp, cc.collision_checker)
        self._dict_time_to_drivable_area = None
        self._dict_time_to_reachable_set = None
        self._dict_time_to_drivable_area_pruned = None
        self._dict_time_to_reachable_set_pruned = None
        self._prune_reachable_set = config.reachable_set.prune_nodes_not_reaching_final_time_step
        self._list_time_steps_computed = list()

    def drivable_area_at_time_step(self, time_step: int) -> List[reach.ReachPolygon]:
        if time_step not in self._list_time_steps_computed:
            print("Given time step for drivable area retrieval is out of range.")
            return []

        else:
            if self._prune_reachable_set:
                return self._dict_time_to_drivable_area_pruned[time_step]

            else:
                return self._dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int) -> List[reach.ReachNode]:
        if time_step not in self._list_time_steps_computed:
            print("Given time step for reachable set retrieval is out of range.")
            return []

        else:
            if self._prune_reachable_set:
                return self._dict_time_to_reachable_set_pruned[time_step]

            else:
                return self._dict_time_to_reachable_set[time_step]

    def compute_reachable_sets(self, time_step_start: int = 1, time_step_end: int = 0):
        """Computes reachable sets for the specified time steps."""
        assert time_step_start != 0, "<CppReachableSetInterface> Time step should not start with 0."

        if not time_step_end:
            time_step_end = self.time_step_end

        for time_step in range(time_step_start, time_step_end + 1):
            self._reach.compute_at_time_step(time_step)

        self._dict_time_to_drivable_area = self._reach.map_time_to_drivable_area
        self._dict_time_to_reachable_set = self._reach.map_time_to_reachable_set
        self._dict_time_to_reachable_set_pruned = self._reach.map_time_to_reachable_set_pruned

        # todo: add pruning in the C++ side
        # if self.config.reachable_set.prune_nodes_not_reaching_final_time_step:
        #     self._prune_reachable_set()
