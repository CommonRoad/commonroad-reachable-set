import logging
import time

from commonroad_reach.data_structure.reach.reach_set import ReachableSet
from commonroad_reach.data_structure.configuration import Configuration

logger = logging.getLogger(__name__)


class ReachableSetInterface:
    """Interface for reachable set computation."""

    def __init__(self, config: Configuration):
        logger.info("Initializing reachable set interface...")

        self.config = config
        self._reach = None
        self._reachable_set_computed = False
        self._initialize_reachable_set()

        logger.info("Reachable set interface initialized.")

    def _initialize_reachable_set(self):
        if self.config.reachable_set.mode in [1, 2, 3, 4, 5]:
            self._reach = ReachableSet.instantiate(self.config)

        else:
            message = "Specified mode ID is invalid."
            logger.error(message)
            raise Exception(message)

    @property
    def time_step_start(self):
        return self._reach.time_step_start

    @property
    def time_step_end(self):
        return self._reach.time_step_end

    @property
    def drivable_area(self):
        return self._reach.drivable_area

    @property
    def reachable_set(self):
        return self._reach.reachable_set

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
        message = "* Computing reachable sets..."
        logger.info(message)
        print(message)

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

        time_start = time.time()
        self._reach.compute(time_step_start, time_step_end)
        self._reachable_set_computed = True
        time_computation = time.time() - time_start

        message = f"\tComputation took: \t{time_computation:.3f}s"
        logger.info(message)
        print(message)

        return time_computation
