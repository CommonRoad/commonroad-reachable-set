import logging

logger = logging.getLogger(__name__)
from typing import List

from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon

from collections import defaultdict
from datetime import datetime

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_analysis_offline import OfflineReachabilityAnalysis
from commonroad_reach.utility import offline_generation as util_offline


class PyGridOfflineReachableSet:
    """Offline step in the multi-step reachable set computation with Python backend."""

    def __init__(self, config: Configuration):
        self.config = config
        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        self._reachability_analysis = OfflineReachabilityAnalysis(config)
        self._dict_time_to_base_set_propagated = defaultdict(list)
        self._dict_time_to_drivable_area = defaultdict(list)
        self._dict_time_to_reachable_set = defaultdict(list)
        self._dict_time_to_drivable_area[self.time_step_start] = self._reachability_analysis.initial_drivable_area
        self._dict_time_to_reachable_set[self.time_step_start] = self._reachability_analysis.initial_reachable_set
        self._list_time_steps_computed = [0]

    def drivable_area_at_time_step(self, time_step: int) -> List[ReachPolygon]:
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for drivable area retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self._dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int) -> List[ReachNodeMultiGeneration]:
        if time_step not in self._list_time_steps_computed:
            message = "Given time step for reachable set retrieval is out of range."
            print(message)
            logger.warning(message)
            return []

        else:
            return self._dict_time_to_reachable_set[time_step]

    def compute_reachable_sets(self, time_step_start: int, time_step_end: int):
        """Computes reachable sets for the specified time steps."""
        for time_step in range(time_step_start, time_step_end + 1):
            print(time_step)

            self._compute_at_time_step(time_step)
            self._list_time_steps_computed.append(time_step)

            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            print(f"time: {current_time}, #nodes: {len(self.reachable_set_at_time_step(time_step))}")

        self._reachability_analysis.determine_grandparent_relationship(self._dict_time_to_reachable_set)

        util_offline.save_offline_computation(self.config, reach_interface)

    def _compute_at_time_step(self, time_step: int):
        """Computes drivable area and reachable set of the time step."""
        self._compute_drivable_area_at_time_step(time_step)
        self._compute_reachable_set_at_time_step(time_step)

    def _compute_drivable_area_at_time_step(self, time_step: int):
        """Computes drivable area of the time step.

        Drivable area is computed based on reachable set of the last time step.
        """
        reachable_set_previous = self._dict_time_to_reachable_set[time_step - 1]
        drivable_area, base_set_propagated = \
            self._reachability_analysis.compute_drivable_area_at_time_step(time_step, reachable_set_previous)

        self._dict_time_to_drivable_area[time_step] = drivable_area
        self._dict_time_to_base_set_propagated[time_step] = base_set_propagated

    def _compute_reachable_set_at_time_step(self, time_step):
        """Compute reachable set of the time step.

        Reachable set is computed based on the drivable area and the list of propagated nodes. It is also based on the
        reachable set of the last time step.
        """
        base_sets_propagated = self._dict_time_to_base_set_propagated[time_step]
        drivable_area = self._dict_time_to_drivable_area[time_step]
        reachable_set = self._reachability_analysis.compute_reachable_set_at_time_step(time_step,
                                                                                       base_sets_propagated,
                                                                                       drivable_area)
        self._dict_time_to_reachable_set[time_step] = reachable_set
