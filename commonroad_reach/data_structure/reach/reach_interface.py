import time
from typing import List, Union

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_interface_py import PyReachableSetInterface
from commonroad_reach.data_structure.reach.reach_node import ReachNode
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


class ReachableSetInterface:
    """Interface to work with reachable sets.

    Both python and C++ backends are supported.
    """

    def __init__(self, config: Configuration):
        self.config = config
        self.mode = config.reachable_set.mode

        # Python backend
        if self.mode == 1 or self.mode == 2:
            self._interface = PyReachableSetInterface(config)

        # C++ backend
        elif self.mode == 3:
            try:
                from commonroad_reach.data_structure.reach.reach_interface_cpp import CppReachableSetInterface

            except ImportError:
                print("<ReachableSetInterface> Cannot import C++ reachable set interface.")

            else:
                self._interface = CppReachableSetInterface(config)

        else:
            raise Exception("<ReachableSetInterface> Specified backend is not valid.")

    @property
    def reachability_analysis(self):
        return self._interface.reachability_analysis

    @property
    def time_step_start(self):
        return self._interface.time_step_start

    @property
    def time_step_end(self):
        return self._interface.time_step_end

    @property
    def reachable_set(self):
        return self._interface.dict_time_to_reachable_set

    @property
    def drivable_area(self):
        return self._interface.dict_time_to_drivable_area

    def drivable_area_at_time_step(self, time_step: int) -> Union[List[ReachPolygon]]:
        return self._interface.drivable_area_at_time_step(time_step)

    def reachable_set_at_time_step(self, time_step: int) -> Union[List[ReachNode]]:
        return self._interface.reachable_set_at_time_step(time_step)

    def pruned_reachable_set_at_time_step(self, time_step: int) -> Union[List[ReachNode]]:
        return self._interface.pruned_reachable_set_at_time_step(time_step)

    def compute_reachable_sets(self, time_step_start: int = 1, time_step_end: int = 0):
        """Computes reachable sets for the specified time steps."""
        print(f"Computing reachable sets...")
        time_start = time.time()
        self._interface.compute_reachable_sets(time_step_start, time_step_end)

        if self.config.debug.measure_time:
            print(f"\tComputation took: \t{time.time() - time_start:.3f}s")
