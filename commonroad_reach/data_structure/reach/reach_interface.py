import logging
import time

from commonroad_reach.data_structure.reach.reach_set import ReachableSet
from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.driving_corridors import DrivingCorridorExtractor

logger = logging.getLogger(__name__)


class ReachableSetInterface:
    """Interface for reachable set computation."""

    def __init__(self, config: Configuration):
        logger.info("Initializing reachable set interface...")

        self.config = config
        self._reach = None
        self._reachable_set_computed = False
        self._initialize_reachable_set()
        self._driving_corridor_extractor = None

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

    def extract_driving_corridors(self, longitudinal_dc=None, longitudinal_positions=None, to_goal_region=False):
        if self._driving_corridor_extractor is None:
            if self.reachable_set is not None:
                self._driving_corridor_extractor = DrivingCorridorExtractor(self.reachable_set, self.config)
            else:
                message = "Reachable sets are empty; please compute reachable sets before extracting driving corridors."
                logger.warning(message)
                print(message)
                return None

        if longitudinal_dc is None or longitudinal_positions is None:
            if to_goal_region:
                goal_region = self.config.planning_problem.goal.state_list[0].position.shapes[0]
                driving_corridors_list = self._driving_corridor_extractor.\
                    extract_lon_driving_corridor(terminal_set=goal_region)
            else:
                driving_corridors_list = self._driving_corridor_extractor.extract_lon_driving_corridor(terminal_set=None)
        else:
            driving_corridors_list = self._driving_corridor_extractor.extract_lat_driving_corridor(
                longitudinal_positions, longitudinal_dc)

        return driving_corridors_list
