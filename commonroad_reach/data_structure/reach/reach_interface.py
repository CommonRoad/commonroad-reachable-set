import logging
import time

logger = logging.getLogger(__name__)
from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_set_py import PyReachableSet
from commonroad_reach.data_structure.reach.reach_set_py_grid_offline import PyGridOfflineReachableSet
from commonroad_reach.data_structure.reach.reach_set_py_grid_online import PyGridOnlineReachableSet


class ReachableSetInterface:
    """Interface for reachable set computation."""

    def __init__(self, config: Configuration):
        logger.info("Initializing reachable set interface...")

        self.config = config
        self.mode = config.reachable_set.mode
        self.time_step_start = self.config.planning.time_step_start
        self.time_step_end = self.time_step_start + self.config.planning.time_steps_computation

        self._reach = None
        self._reachable_set_computed = False
        self._initialize_reachable_set()

        logger.info("Reachable set interface initialized.")

    def _initialize_reachable_set(self):
        if self.mode in [1, 2]:
            self._reach = PyReachableSet(self.config)

        elif self.mode == 3:
            try:
                from commonroad_reach.data_structure.reach.reach_set_cpp import CppReachableSet

            except ImportError:
                message = "Importing C++ reachable set failed."
                logger.exception(message)
                print(message)

            else:
                self._reach = CppReachableSet(self.config)

        elif self.mode in [4, 5]:
            self._reach = PyGridOnlineReachableSet(self.config)

        elif self.mode == 6:
            self._reach = PyGridOfflineReachableSet(self.config)

        else:
            message = "Specified mode ID is invalid."
            logger.error(message)
            raise Exception(message)

    def drivable_area_at_time_step(self, time_step: int):
        if not self._reachable_set_computed and time_step != 0:
            message = "Reachable set is not computed, retrieving drivable area failed."
            logger.warning(message)
            print(message)
            return []

        else:
            return self._reach.drivable_area_at_time_step(time_step)

    def reachable_set_at_time_step(self, time_step: int):
        if not self._reachable_set_computed and time_step != 0:
            message = "Reachable set is not computed, retrieving reachable set failed."
            logger.warning(message)
            print(message)
            return []

        else:
            return self._reach.reachable_set_at_time_step(time_step)

    def compute_reachable_sets(self, time_step_start: int = 1, time_step_end: int = 0):
        """Calls reachable set computation functions for the specified time steps."""
        if not self._reach:
            message = "Reachable set is not initialized, aborting computation."
            logger.warning(message)
            print(message)
            return None

        if not time_step_end:
            time_step_end = self.time_step_end

        if not (0 < time_step_start < time_step_end):
            message = "Time steps for computation are invalid, aborting computation."
            logger.warning(message)
            print(message)
            return None

        message = "* Computing reachable sets..."
        logger.info(message)
        print(message)

        time_start = time.time()
        self._reach.compute_reachable_sets(time_step_start, time_step_end)

        message = f"\tComputation took: \t{time.time() - time_start:.3f}s"
        logger.info(message)
        print(message)

        self._reachable_set_computed = True
