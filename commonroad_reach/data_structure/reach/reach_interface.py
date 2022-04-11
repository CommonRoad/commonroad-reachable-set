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
        if self.config.reachable_set.mode_computation in [1, 2, 3, 4]:
            self._reach = ReachableSet.instantiate(self.config)

        else:
            message = "Specified mode ID is invalid."
            logger.error(message)
            raise Exception(message)

    @property
    def step_start(self):
        return self._reach.step_start

    @property
    def step_end(self):
        return self._reach.step_end

    @property
    def drivable_area(self):
        return self._reach.drivable_area

    @property
    def reachable_set(self):
        return self._reach.reachable_set

    def drivable_area_at_step(self, step: int):
        if not self._reachable_set_computed and step != 0:
            message = "Reachable set is not computed, retrieving drivable area failed."
            logger.warning(message)
            print(message)
            return []

        else:
            return self._reach.drivable_area_at_step(step)

    def reachable_set_at_step(self, step: int):
        if not self._reachable_set_computed and step != 0:
            message = "Reachable set is not computed, retrieving reachable set failed."
            logger.warning(message)
            print(message)
            return []

        else:
            return self._reach.reachable_set_at_step(step)

    def compute_reachable_sets(self, step_start: int = 1, step_end: int = 0):
        message = "* Computing reachable sets..."
        logger.info(message)
        print(message)

        if not self._reach:
            message = "Reachable set is not initialized, aborting computation."
            logger.warning(message)
            print(message)
            return None

        step_end = step_end if step_end else self.step_end

        if not (0 < step_start < step_end):
            message = "Time steps for computation are invalid, aborting computation."
            logger.warning(message)
            print(message)
            return None

        time_start = time.time()
        self._reach.compute(step_start, step_end)
        self._reachable_set_computed = True
        time_computation = time.time() - time_start

        message = f"\tComputation took: \t{time_computation:.3f}s"
        logger.info(message)
        print(message)

        # return computation time for benchmarking
        # return time_computation
