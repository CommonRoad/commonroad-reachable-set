from typing import List

import pycrreachset as reach

from commonroad_reachset.data_structure.collision_checker import CollisionChecker
from commonroad_reachset.data_structure.configuration import Configuration
from commonroad_reachset.data_structure.reach.reach_node import ReachNode
from commonroad_reachset.data_structure.reach.reach_polygon import ReachPolygon


class CppReachableSetInterface:
    """Interface to work with reachable sets (C++ backend)."""

    @classmethod
    def continuous(cls, config: Configuration):
        config_cpp = config.convert_to_cpp_configuration()
        collision_checker = CollisionChecker(config)

        reachability_analysis = reach.ReachabilityAnalysis(config_cpp,
                                                           collision_checker.collision_checker_cpp)

        return CPPReachableSetInterface(reachability_analysis)

    def __init__(self, reachability_analysis: reach.ReachabilityAnalysis):
        self._reachability_analysis = reachability_analysis

        self._time_step_initial = reachability_analysis.config().planning.time_step_start
        self._dict_time_to_drivable_area = dict()
        self._dict_time_to_list_base_sets_propagated = dict()
        self._dict_time_to_reachable_set = dict()

        self._initialize()

    @property
    def config(self) -> Configuration:
        return self._reachability_analysis.config()

    @property
    def dict_time_to_drivable_area(self) -> Dict[int, List[ReachPolygon]]:
        """Dictionary holding drivable area at different time steps."""
        return self._dict_time_to_drivable_area

    @property
    def dict_time_to_reachable_set(self) -> Dict[int, List[ReachNode]]:
        """Dictionary holding reachable set at different time steps."""
        return self._dict_time_to_reachable_set

    def _initialize(self):
        """Initializes drivable area and reachable set of initial time step."""
        self._dict_time_to_drivable_area[
            self._time_step_initial
        ] = self._reachability_analysis.initial_drivable_area()

        self._dict_time_to_reachable_set[
            self._time_step_initial
        ] = self._reachability_analysis.initial_reachable_set()

    def compute(self, time_step_start: int = 1, time_step_end: int = 0):
        """Compute reachable set between the specified time steps."""
        if time_step_start == 0:
            print("Time step should not start with 0.")
            return

        if not time_step_end:
            time_step_end = self.config.planning.time_steps_computation

        for time_step in range(time_step_start, time_step_end + 1):
            self._compute_at_time_step(time_step)

    def _compute_at_time_step(self, time_step: int):
        """Compute drivable area and reachable set of the given time step."""
        self._compute_drivable_area_at_time_step(time_step)
        self._compute_reachable_set_at_time_step(time_step)

    def _compute_drivable_area_at_time_step(self, time_step: int):
        """Compute drivable area of the time step.

        Drivable area is computed based on reachable set of the last time step.
        """
        # prepare
        reachable_set_time_step_last = self._dict_time_to_reachable_set[time_step - 1]
        # compute
        drivable_area, list_base_sets_propagated = \
            self._reachability_analysis.compute_drivable_area_at_time_step(time_step, reachable_set_time_step_last)

        # save
        self._dict_time_to_drivable_area[time_step] = drivable_area
        self._dict_time_to_list_base_sets_propagated[time_step] = list_base_sets_propagated

    def _compute_reachable_set_at_time_step(self, time_step):
        """Compute reachable set of the time step.

        Reachable set is computed based on the drivable area and the list of
        propagated nodes. It is also based on the reachable set of the last time
        step.
        """
        # prepare
        reachable_set_time_step_previous = self._dict_time_to_reachable_set[time_step - 1]
        list_base_sets_propagated = self._dict_time_to_list_base_sets_propagated[time_step]
        drivable_area = self._dict_time_to_drivable_area[time_step]
        # compute
        reachable_set = self._reachability_analysis.compute_reachable_set_at_time_step(
            time_step,
            reachable_set_time_step_previous,
            list_base_sets_propagated,
            drivable_area,
        )
        # save
        self._dict_time_to_reachable_set[time_step] = reachable_set
