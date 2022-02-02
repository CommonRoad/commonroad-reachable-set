import logging

logger = logging.getLogger(__name__)
from abc import ABC, abstractmethod
from collections import defaultdict

from commonroad_reach.data_structure.configuration import Configuration


class ReachableSet(ABC):
    """Abstract class for reachable set."""

    def __init__(self, config: Configuration):
        self.config = config
        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        self._dict_time_to_base_set_propagated = defaultdict(list)
        self._dict_time_to_drivable_area = defaultdict(list)
        self._dict_time_to_reachable_set = defaultdict(list)

        self._prune_reachable_set = config.reachable_set.prune_nodes_not_reaching_final_time_step
        self._pruned = False

        self._list_time_steps_computed = [0]

    def drivable_area_at_time_step(self, time_step: int):
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for drivable area retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self._dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int):
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for reachable set retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self._dict_time_to_reachable_set[time_step]

    @abstractmethod
    def compute_reachable_sets(self, time_step_start: int, time_step_end: int):
        """Calls reachable set computation functions for the specified time steps."""
        pass

    @abstractmethod
    def _prune_nodes_not_reaching_final_time_step(self):
        """Prunes nodes that don't reach the final time step.

        Iterates through reachability graph backward in time, discards nodes that don't have a child node.
        """
        pass

    @classmethod
    def instantiate(cls, config: Configuration):
        """Instantiates a reachable set class based on the given configuration."""
        mode = config.reachable_set.mode

        if mode in [1, 2]:
            from commonroad_reach.data_structure.reach.reach_set_py import PyReachableSet
            return PyReachableSet(config)

        elif mode == 3:
            try:
                from commonroad_reach.data_structure.reach.reach_set_cpp import CppReachableSet

            except ImportError:
                message = "Importing C++ reachable set failed."
                logger.exception(message)
                print(message)

            else:
                return CppReachableSet(config)

        elif mode in [4, 5]:
            from commonroad_reach.data_structure.reach.reach_set_py_grid_online import PyGridOnlineReachableSet
            return PyGridOnlineReachableSet(config)

        elif mode == 6:
            from commonroad_reach.data_structure.reach.reach_set_py_grid_offline import PyGridOfflineReachableSet
            return PyGridOfflineReachableSet(config)
