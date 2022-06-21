import time
import logging

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.driving_corridors import DrivingCorridorExtractor
from commonroad_reach.data_structure.reach.reach_set import ReachableSet
from commonroad.geometry.shape import ShapeGroup

logger = logging.getLogger(__name__)


class ReachableSetInterface:
    """Interface for reachable set computation."""

    def __init__(self, config: Configuration, logger_level=logging.DEBUG):
        # todo: should this go into config or set in the logger module? it doesn't seem appropriate here
        logger.setLevel(logger_level)
        logger.info("Initializing reachable set interface...")

        self.config = None
        self._reach = None
        self._reachable_set_computed = False
        self._driving_corridor_extractor = None

        self.reset(config)

        logger.info("Reachable set interface initialized.")

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

    def reset(self, config: Configuration):
        """Resets configuration."""
        self.config = config
        self._reachable_set_computed = False
        self._driving_corridor_extractor = None

        if self.config.reachable_set.mode_computation in [1, 2, 3, 4]:
            self._reach = ReachableSet.instantiate(self.config)

        else:
            message = "Specified mode ID is invalid."
            logger.error(message)
            raise Exception(message)

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
        logger.debug(message)

        if not self._reach:
            message = "Reachable set is not initialized, aborting computation."
            logger.warning(message)
            return None

        step_end = step_end if step_end else self.step_end

        if not (0 < step_start < step_end):
            message = "Time steps for computation are invalid, aborting computation."
            logger.warning(message)
            return None

        time_start = time.time()
        self._reach.compute(step_start, step_end)
        self._reachable_set_computed = True
        time_computation = time.time() - time_start

        message = f"\tComputation took: \t{time_computation:.3f}s"
        print(message)
        logger.debug(message)

    def extract_driving_corridors(self, corridor_lon=None, list_p_lon=None,
                                  to_goal_region=False, shape_terminal=None, is_cartesian_shape: bool = True):
        """Extracts driving corridors from the computed reachable sets.

        If a longitudinal DC and a list of longitudinal positions are given, extracts the lateral DCs. Otherwise,
        proceed to longitudinal DC extraction. Optionally, one can specify whether the longitudinal DC should reach
        the goal region of the planning problem or a user-given terminal state represented by a shape.
        """
        if not self.reachable_set:
            message = "Reachable sets are empty! Compute reachable sets before extracting driving corridors."
            print(message)
            logger.warning(message)

            return None

        if not self._driving_corridor_extractor:
            self._driving_corridor_extractor = DrivingCorridorExtractor(self.reachable_set, self.config)

        print(f"*** Extracting driving corridors...")
        time_start = time.time()
        list_corridors = list()
        if not (corridor_lon and list_p_lon):
            # extract longitudinal driving corridors
            if to_goal_region:
                # extract all driving corridors reaching the goal region represented by a shape(group)
                shape_goal = self.config.planning_problem.goal.state_list[0].position
                if isinstance(shape_goal, ShapeGroup):
                    list_shapes_goal = [shape for shape in shape_goal.shapes]

                else:
                    list_shapes_goal = [shape_goal]

                for shape_goal in list_shapes_goal:
                    list_corridors += \
                        self._driving_corridor_extractor.extract_driving_corridors(shape_terminal=shape_goal,
                                                                                   is_cartesian_shape=is_cartesian_shape)
            else:
                list_corridors = \
                    self._driving_corridor_extractor.extract_driving_corridors(shape_terminal=shape_terminal,
                                                                               is_cartesian_shape=is_cartesian_shape)

        else:
            # extract lateral driving corridors
            list_corridors = \
                self._driving_corridor_extractor.extract_driving_corridors(corridor_lon=corridor_lon,
                                                                           list_p_lon=list_p_lon)

        print(f"\tDC extraction took: \t{time.time() - time_start:.3f}s")

        return list_corridors
