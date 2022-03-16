import logging

from commonroad_reach.data_structure.reach.reach_set import ReachableSet

logger = logging.getLogger(__name__)

import pycrreach as reach

from commonroad_reach.data_structure.collision_checker_cpp import CppCollisionChecker
from commonroad_reach.data_structure.configuration import Configuration


class CppReachableSet(ReachableSet):
    """Reachable set computation with C++ backend."""

    def __init__(self, config: Configuration):
        super().__init__(config)
        config_cpp = self.config.convert_to_cpp_configuration()

        cc = CppCollisionChecker(self.config)
        self._reach = reach.ReachableSetInterface(config_cpp, cc.collision_checker)

    def compute_reachable_sets(self, time_step_start: int, time_step_end: int):
        """Computes reachable sets for the specified time steps."""
        for time_step in range(time_step_start, time_step_end + 1):
            logger.debug(f"Computing reachable set for time step {time_step}")
            self._reach.compute_at_time_step(time_step)
            self._list_time_steps_computed.append(time_step)

        self._dict_time_to_drivable_area = self._reach.map_time_to_drivable_area
        self._dict_time_to_reachable_set = self._reach.map_time_to_reachable_set

        if self.config.reachable_set.prune_nodes_not_reaching_final_time_step:
            self._prune_nodes_not_reaching_final_time_step()

    def _prune_nodes_not_reaching_final_time_step(self):
        # todo: add pruning in the C++ side
        pass
