from typing import List

import pycrreachset as reach

from commonroad_reachset.data_structure.collision_checker_cpp import CppCollisionChecker
from commonroad_reachset.data_structure.configuration import Configuration


class CppReachableSetInterface:
    """Interface to work with C++ backend."""

    def __init__(self, config: Configuration):
        self.config = config
        config_cpp = config.convert_to_cpp_configuration()
        collision_checker = CppCollisionChecker(config)

        self.reach_interface = reach.ReachableSetInterface(config_cpp, collision_checker.collision_checker)
        self.reachability_analysis = self.reach_interface.reachability_analysis
        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        self.dict_time_to_drivable_area = None
        self.dict_time_to_reachable_set = None
        self.dict_time_to_reachable_set_pruned = None

    def drivable_area_at_time_step(self, time_step: int) -> List[reach.ReachPolygon]:
        # assert time_step in self.dict_time_to_drivable_area, "<CppReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int) -> List[reach.ReachNode]:
        # assert time_step in self.dict_time_to_drivable_area, "<CppReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_reachable_set[time_step]

    def pruned_reachable_set_at_time_step(self, time_step: int) -> List[reach.ReachNode]:
        # assert time_step in self.dict_time_to_drivable_area, "<CppReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_reachable_set_pruned[time_step]

    def compute_reachable_sets(self, time_step_start: int = 1, time_step_end: int = 0):
        """Computes reachable sets for the specified time steps."""
        assert time_step_start != 0, "<CppReachableSetInterface> Time step should not start with 0."

        if not time_step_end:
            time_step_end = self.time_step_end

        for time_step in range(time_step_start, time_step_end + 1):
            self.reach_interface.compute_at_time_step(time_step)

        self.dict_time_to_drivable_area = self.reach_interface.map_time_to_drivable_area
        self.dict_time_to_reachable_set = self.reach_interface.map_time_to_reachable_set
        self.dict_time_to_reachable_set_pruned = self.reach_interface.map_time_to_reachable_set_pruned

        # todo: add pruning in the C++ side
        # if self.config.reachable_set.prune_nodes_not_reaching_final_time_step:
        #     self._prune_reachable_set()
