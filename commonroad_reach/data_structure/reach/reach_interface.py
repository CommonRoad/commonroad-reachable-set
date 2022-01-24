import time

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_set_py import PyReachableSet
from commonroad_reach.data_structure.reach.reach_set_py_grid_online import PyGridOnlineReachableSet


class ReachableSetInterface:
    """Interface to work with reachable sets.

    Both python and C++ backends are supported.
    """

    def __init__(self, config: Configuration):
        self.config = config
        self.mode = config.reachable_set.mode
        self.time_step_start = self.config.planning.time_step_start
        self.time_step_end = self.time_step_start + self.config.planning.time_steps_computation

        self._reach = None
        self._reachable_set_computed = False
        self._initialize_reachable_set()

    def _initialize_reachable_set(self):
        # Single-step with Python backend
        if self.mode in [1, 2]:
            self._reach = PyReachableSet(self.config)

        # Single-step with C++ backend
        elif self.mode == 3:
            try:
                from commonroad_reach.data_structure.reach.reach_set_cpp import CppReachableSet

            except ImportError:
                print("Importing C++ reachable set failed.")

            else:
                self._reach = CppReachableSet(self.config)

        # Multi-step with Python backend
        elif self.mode in [4, 5]:
            self._reach = PyGridOnlineReachableSet(self.config)

        else:
            raise Exception("Specified mode ID is invalid.")

    def drivable_area_at_time_step(self, time_step: int):
        if not self._reachable_set_computed and time_step != 0:
            print("Reachable set is not computed, retrieving drivable area failed.")
            return []

        else:
            return self._reach.drivable_area_at_time_step(time_step)

    def reachable_set_at_time_step(self, time_step: int):
        if not self._reachable_set_computed and time_step != 0:
            print("<ReachableSetInterface> Reachable set is not computed, retrieving reachable set failed.")
            return []

        else:
            return self._reach.reachable_set_at_time_step(time_step)

    def compute_reachable_sets(self, time_step_start: int = 1, time_step_end: int = 0):
        """Computes reachable sets for the specified time steps."""
        if not self._reach:
            print("Reachable set is not initialized, aborting computation.")

        else:
            print(f"Computing reachable sets...")
            time_start = time.time()
            self._reach.compute_reachable_sets(time_step_start, time_step_end)

            if self.config.debug.measure_time:
                print(f"\tComputation took: \t{time.time() - time_start:.3f}s")

            self._reachable_set_computed = True
