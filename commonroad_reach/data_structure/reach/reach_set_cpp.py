import logging

logger = logging.getLogger(__name__)
from typing import List

import pycrreachset as reach

from commonroad_reach.data_structure.collision_checker_cpp import CppCollisionChecker
from commonroad_reach.data_structure.configuration import Configuration


class CppReachableSet:
    """Reachable set computation with C++ backend."""

    def __init__(self, config: Configuration):
        self.config = config
        config_cpp = config.convert_to_cpp_configuration()
        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        cc = CppCollisionChecker(config)
        self._reach = reach.ReachableSetInterface(config_cpp, cc.collision_checker)
        self._dict_time_to_drivable_area = None
        self._dict_time_to_reachable_set = None
        self._prune_reachable_set = config.reachable_set.prune_nodes_not_reaching_final_time_step
        self._list_time_steps_computed = [0]

    def drivable_area_at_time_step(self, time_step: int) -> List[reach.ReachPolygon]:
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for drivable area retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self._dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int) -> List[reach.ReachNode]:
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for reachable set retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self._dict_time_to_reachable_set[time_step]

    def compute_reachable_sets(self, time_step_start: int, time_step_end: int):
        """Computes reachable sets for the specified time steps."""
        # todo: prune, update dict
        for time_step in range(time_step_start, time_step_end + 1):
            logger.debug(f"Computing reachable set for time step {time_step}")
            self._reach.compute_at_time_step(time_step)
            self._list_time_steps_computed.append(time_step)

        self._dict_time_to_drivable_area = self._reach.map_time_to_drivable_area
        self._dict_time_to_reachable_set = self._reach.map_time_to_reachable_set

        # todo: add pruning in the C++ side
        # if self.config.reachable_set.prune_nodes_not_reaching_final_time_step:
        #     self._prune_reachable_set()
