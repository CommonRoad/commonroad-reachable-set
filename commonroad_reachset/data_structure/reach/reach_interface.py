# from commonroad_reachset.common.collision_checker import CollisionChecker
import time
from typing import List, Union

from commonroad_reachset.data_structure.configuration import Configuration
from commonroad_reachset.data_structure.reach.reach_interface_py import PyReachableSetInterface
from commonroad_reachset.data_structure.reach.reach_node import ReachNode
from commonroad_reachset.data_structure.reach.reach_polygon import ReachPolygon


class ReachableSetInterface:
    """Interface to work with reachable sets."""

    def __init__(self, config: Configuration, back_end=None):
        self.config = config

        back_end = back_end or config.reachable_set.back_end
        if back_end == "PYTHON":
            self.back_end = "PYTHON"
            self._interface = PyReachableSetInterface(config)

        elif back_end == "CPP":
            try:
                from commonroad_reachset.data_structure.reach.reach_interface_py import CppReachableSetInterface

            except ImportError:
                print("<ReachableSetInterface> Cannot import C++ reachable set interface.")

            else:
                self.back_end = "CPP"
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
