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
        self._reach = reach.ReachableSet(self.config.convert_to_cpp_configuration(),
                                         CppCollisionChecker(self.config).collision_checker)

        logger.info("CppReachableSet initialized.")

    def compute(self, time_step_start: int, time_step_end: int):
        """Computes reachable sets for the specified time steps."""
        for time_step in range(time_step_start, time_step_end + 1):
            logger.debug(f"Computing reachable set for time step {time_step}")
            self._reach.compute(time_step, time_step)
            self._list_time_steps_computed.append(time_step)

        self.dict_time_to_drivable_area = self._reach.drivable_area()
        self.dict_time_to_reachable_set = self._reach.reachable_set()

        if self.config.reachable_set.prune_nodes_not_reaching_final_time_step:
            self._prune_nodes_not_reaching_final_time_step()

    def _prune_nodes_not_reaching_final_time_step(self):
        self._reach.prune_nodes_not_reaching_final_time_step()
