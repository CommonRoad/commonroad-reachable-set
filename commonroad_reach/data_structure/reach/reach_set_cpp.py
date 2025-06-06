import logging

from commonroad_reach.data_structure.reach.reach_set import ReachableSet

logger = logging.getLogger("REACH_LOGGER")

import commonroad_reach.pycrreach as reach
from commonroad_reach.data_structure.collision_checker import CollisionChecker
from commonroad_reach.data_structure.configuration import Configuration


class CppReachableSet(ReachableSet):
    """
    Reachable set computation with C++ backend.
    """

    def __init__(self, config: Configuration):
        super().__init__(config)
        self.collision_checker = CollisionChecker(self.config)
        self._reach = reach.ReachableSet(self.config.convert_to_cpp_configuration(),
                                         self.collision_checker.cpp_collision_checker)

        logger.debug("CppReachableSet initialized.")

    def compute(self, step_start: int, step_end: int):
        for step in range(step_start, step_end + 1):
            logger.debug(f"Computing reachable set for step {step}")
            self._reach.compute(step, step)
            self._list_steps_computed.append(step)

        self.dict_step_to_drivable_area = self._reach.drivable_area()
        self.dict_step_to_reachable_set = self._reach.reachable_set()

        if self.config.reachable_set.prune_nodes_not_reaching_final_step:
            self.prune_nodes_not_reaching_final_step()

    def compute_drivable_area_at_step(self, step):
        pass

    def compute_reachable_set_at_step(self, step):
        pass

    def _reset_reachable_set_at_step(self, step: int, reachable_set):
        pass

    def prune_nodes_not_reaching_final_step(self):
        self._reach.prune_nodes_not_reaching_final_step()
