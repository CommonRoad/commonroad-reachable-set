import logging
from abc import ABC, abstractmethod
from collections import defaultdict

from commonroad_reach.data_structure.configuration import Configuration
import commonroad_reach.utility.logger as util_logger

logger = logging.getLogger("REACH_LOGGER")


class ReachableSet(ABC):
    """
    Abstract superclass for reachable sets.
    """

    def __init__(self, config: Configuration):
        self.config: Configuration = config
        self.collision_checker = None

        self.step_start = config.planning.step_start
        self.step_end = config.planning.steps_computation + self.step_start

        self.dict_step_to_propagated_set = defaultdict(list)
        self.dict_step_to_drivable_area = defaultdict(list)
        self.dict_step_to_reachable_set = defaultdict(list)

        self._prune_reachable_set = config.reachable_set.prune_nodes_not_reaching_final_step
        self._pruned = False

        self._list_steps_computed = [self.step_start]

    @property
    def propagated_set(self):
        return self.dict_step_to_propagated_set

    @property
    def drivable_area(self):
        return self.dict_step_to_drivable_area

    @property
    def reachable_set(self):
        return self.dict_step_to_reachable_set

    def propagated_set_at_step(self, step: int):
        if step not in self._list_steps_computed:
            util_logger.print_and_log_warning(logger,
                                              f"Given step {step} for propagated set retrieval is out of range.")
            return []

        else:
            return self.dict_step_to_propagated_set[step]

    def drivable_area_at_step(self, step: int):
        if step not in self._list_steps_computed:
            util_logger.print_and_log_warning(logger,
                                              f"Given step {step} for drivable area retrieval is out of range.")
            return []

        else:
            return self.dict_step_to_drivable_area[step]

    def reachable_set_at_step(self, step: int):
        if step not in self._list_steps_computed:
            util_logger.print_and_log_warning(logger,
                                              f"Given step {step} for reachable set retrieval is out of range.")
            return []

        else:
            return self.dict_step_to_reachable_set[step]

    def reset_drivable_area_at_step(self, step: int, drivable_area):
        if step not in self._list_steps_computed:
            util_logger.print_and_log_warning(logger,
                                              f"Given step {step} for resetting drivable area is out of range.")

        else:
            self.dict_step_to_drivable_area[step] = drivable_area

    def reset_reachable_set_at_step(self, step: int, reachable_set):
        if step not in self._list_steps_computed:
            util_logger.print_and_log_warning(logger,
                                              f"Given step {step} for resetting reachable set is out of range.")

        else:
            self._reset_reachable_set_at_step(step, reachable_set)

    @abstractmethod
    def _reset_reachable_set_at_step(self, step: int, reachable_set):
        pass

    @abstractmethod
    def compute(self, step_start: int, step_end: int):
        """
        Computes reachable set between the specified start and end steps.
        """
        pass

    @abstractmethod
    def compute_drivable_area_at_step(self, step):
        """
        Computes drivable area at a specific step.
        """
        pass

    @abstractmethod
    def compute_reachable_set_at_step(self, step):
        """
        Computes reachable set at a specific step.
        """
        pass

    @abstractmethod
    def prune_nodes_not_reaching_final_step(self):
        """
        Prunes nodes that do not reach the final step.

        Iterates through reachability graph backward in time, discards nodes that do not have a child node.
        """
        pass

    @classmethod
    def instantiate(cls, config: Configuration):
        """
        Instantiates a reachable set class based on the given configuration.
        """
        mode = config.reachable_set.mode_computation

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
