import logging

logger = logging.getLogger(__name__)
from abc import ABC, abstractmethod
from collections import defaultdict

from commonroad_reach.data_structure.configuration import Configuration


class ReachableSet(ABC):
    """Abstract superclass for reachable sets."""

    def __init__(self, config: Configuration):
        self.config: Configuration = config

        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        self.dict_time_to_base_set_propagated = defaultdict(list)
        self.dict_time_to_drivable_area = defaultdict(list)
        self.dict_time_to_reachable_set = defaultdict(list)

        self._prune_reachable_set = config.reachable_set.prune_nodes_not_reaching_final_time_step
        self._pruned = False

        self._list_time_steps_computed = [0]

    @property
    def drivable_area(self):
        return self.dict_time_to_drivable_area

    @property
    def reachable_set(self):
        return self.dict_time_to_reachable_set

    @property
    def max_evaluated_time_step(self):
        return max(self.dict_time_to_reachable_set)

    def drivable_area_at_time_step(self, time_step: int):
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for drivable area retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self.dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int):
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for reachable set retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self.dict_time_to_reachable_set[time_step]

    @abstractmethod
    def compute(self, time_step_start: int, time_step_end: int):
        """Calls reachable set computation for the specified time steps."""
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

        if mode == 1:
            # Polytopic set propagation with Python backend
            from commonroad_reach.data_structure.reach.reach_set_py import PyReachableSet
            return PyReachableSet(config)

        elif mode == 2:
            # Polytopic set propagation with C++ backend
            from commonroad_reach.data_structure.reach.reach_set_cpp import CppReachableSet
            return CppReachableSet(config)

        elif mode == 3:
            # Online stage of the Graph-based propagation
            from commonroad_reach.data_structure.reach.reach_set_py_graph_online import PyGraphReachableSetOnline
            return PyGraphReachableSetOnline(config)

        elif mode == 4:
            # Offline stage of the Graph-based propagation
            from commonroad_reach.data_structure.reach.reach_set_py_graph_offline import PyGraphReachableSetOffline
            return PyGraphReachableSetOffline(config)
