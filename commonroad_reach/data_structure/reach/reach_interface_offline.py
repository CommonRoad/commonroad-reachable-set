from datetime import datetime
from typing import List

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_generator_offline import OfflineReachableSetGenerator
from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


class OfflineReachableSetInterface:
    """Interface to work with reachable sets with python backend."""

    def __init__(self, config: Configuration):
        self.config = config
        self._generator = OfflineReachableSetGenerator(config)
        self.time_step_start = config.planning.time_step_start
        self.time_step_end = config.planning.time_steps_computation + self.time_step_start

        self.dict_time_to_base_set_propagated = dict()
        self.dict_time_to_drivable_area = dict()
        self.dict_time_to_reachable_set = dict()

        self._initialize()

    def _initialize(self):
        """Initializes drivable area and reachable set of the initial time step."""
        self.dict_time_to_drivable_area[self.time_step_start] = self._generator.initial_drivable_area
        self.dict_time_to_reachable_set[self.time_step_start] = self._generator.initial_reachable_set

    def drivable_area_at_time_step(self, time_step: int) -> List[ReachPolygon]:
        assert time_step in self.dict_time_to_drivable_area, "<OfflineReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_drivable_area[time_step]

    def reachable_set_at_time_step(self, time_step: int) -> List[ReachNodeMultiGeneration]:
        assert time_step in self.dict_time_to_drivable_area, "<OfflineReachableSetInterface> Input time step is not valid."

        return self.dict_time_to_reachable_set[time_step]

    def compute_reachable_sets(self, time_step_start: int = 1, time_step_end: int = 0):
        """Computes reachable sets for the specified time steps."""
        assert time_step_start != 0, "<OfflineReachableSetInterface> Time step should not start with 0."

        if not time_step_end:
            time_step_end = self.time_step_end

        for time_step in range(time_step_start, time_step_end + 1):
            print(time_step)
            self._compute_at_time_step(time_step)
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            print(f"time: {current_time}, #nodes: {len(self.reachable_set_at_time_step(time_step))}")

        self._generator.determine_grandparent_relationship(self.dict_time_to_reachable_set)
        self._print_analysis()

    def _compute_at_time_step(self, time_step: int):
        """Computes drivable area and reachable set of the time step."""
        self._compute_drivable_area_at_time_step(time_step)
        self._compute_reachable_set_at_time_step(time_step)

    def _compute_drivable_area_at_time_step(self, time_step: int):
        """Computes drivable area of the time step.

        Drivable area is computed based on reachable set of the last time step.
        """
        reachable_set_previous = self.dict_time_to_reachable_set[time_step - 1]
        drivable_area, base_set_propagated = \
            self._generator.compute_drivable_area_at_time_step(time_step, reachable_set_previous)

        self.dict_time_to_drivable_area[time_step] = drivable_area
        self.dict_time_to_base_set_propagated[time_step] = base_set_propagated

    def _compute_reachable_set_at_time_step(self, time_step):
        """Compute reachable set of the time step.

        Reachable set is computed based on the drivable area and the list of propagated nodes. It is also based on the
        reachable set of the last time step.
        """
        base_sets_propagated = self.dict_time_to_base_set_propagated[time_step]
        drivable_area = self.dict_time_to_drivable_area[time_step]

        reachable_set = self._generator. \
            compute_reachable_set_at_time_step(time_step, base_sets_propagated, drivable_area)

        self.dict_time_to_reachable_set[time_step] = reachable_set

    def _print_analysis(self):
        sum_nodes = 0
        for time_step in self.dict_time_to_reachable_set:
            sum_nodes += len(self.reachable_set_at_time_step(time_step))

        print(f"#nodes: {sum_nodes}")
