import logging
import time
from typing import List, Union
from pathlib import Path

from commonroad.geometry.shape import Shape

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.driving_corridor import DrivingCorridor
from commonroad_reach.data_structure.reach.driving_corridor_extractor import DrivingCorridorExtractor
from commonroad_reach.data_structure.reach.reach_set import ReachableSet
import commonroad_reach.utility.logger as util_logger

logger = logging.getLogger("REACH_LOGGER")


class ReachableSetInterface:
    """
    Interface for reachable set computation.
    """

    def __init__(self, config: Union[Configuration, None]):
        self.config = None
        self._reach = None
        self._reachable_set_computed = False
        self._driving_corridor_extractor = None

        if config is not None:
            self.reset(config)

        util_logger.print_and_log_debug(logger, "Reachable set interface initialized.")

    @property
    def step_start(self):
        return self._reach.step_start

    @property
    def step_end(self):
        return self._reach.step_end

    @property
    def propagated_set(self):
        return self._reach.propagated_set

    @property
    def drivable_area(self):
        return self._reach.drivable_area

    @property
    def reachable_set(self):
        return self._reach.reachable_set

    @property
    def collision_checker(self):
        return self._reach.collision_checker.cpp_collision_checker

    def reset(self, config: Configuration):
        """
        Resets configuration of the interface.
        """
        self.config = config
        self._reach = None
        self._reachable_set_computed = False
        self._driving_corridor_extractor = None

        if self.config.reachable_set.mode_computation in [1, 2, 3, 4]:
            self._reach = ReachableSet.instantiate(self.config)

        else:
            message = "Specified mode ID is invalid."
            util_logger.print_and_log_error(logger, message)
            raise Exception(message)

    def propagated_set_at_step(self, step: int):
        if not self._reachable_set_computed and step != 0:
            util_logger.print_and_log_warning(logger,
                                              "Reachable set is not computed, retrieving propagated set failed.")
            return []

        else:
            return self._reach.propagated_set_at_step(step)

    def drivable_area_at_step(self, step: int):
        if not self._reachable_set_computed and step != 0:
            util_logger.print_and_log_warning(logger, "Reachable set is not computed, retrieving drivable area failed.")
            return []

        else:
            return self._reach.drivable_area_at_step(step)

    def reset_drivable_area_at_step(self, step: int, drivable_area):
        if not self._reachable_set_computed and step != 0:
            util_logger.print_and_log_warning(logger, "Reachable set is not computed, resetting drivable area failed.")
            return []

        else:
            return self._reach.reset_drivable_area_at_step(step, drivable_area)

    def reachable_set_at_step(self, step: int):
        if not self._reachable_set_computed and step != 0:
            util_logger.print_and_log_warning(logger, "Reachable set is not computed, retrieving reachable set failed.")
            return []

        else:
            return self._reach.reachable_set_at_step(step)

    def reset_reachable_set_at_step(self, step: int, reachable_set):
        if not self._reachable_set_computed and step != 0:
            util_logger.print_and_log_warning(logger, "Reachable set is not computed, resetting reachable set failed.")
            return []

        else:
            return self._reach.reset_reachable_set_at_step(step, reachable_set)

    def compute_reachable_sets(self, step_start: int = 0, step_end: int = 0, verbose=True):
        """
        Computes reachable sets between the given start and end steps.
        """
        if not self._reach:
            util_logger.print_and_log_warning(logger, "Reachable set is not initialized, aborting computation.")
            return None

        step_start = step_start if step_start else self.step_start + 1
        step_end = step_end if step_end else self.step_end

        if not (0 < step_start <= step_end):
            util_logger.print_and_log_warning(logger, "Steps for computation are invalid, aborting computation.")
            return None

        # print initial state
        util_logger.print_and_log_info(logger, "* Reach Initial State...", verbose)
        init_state_msg = f"\tTime step: {self.config.planning.step_start}\n"
        init_state_msg += f"\tPosition (lon/lat): {self.config.planning.p_initial}\n"
        init_state_msg += f"\tVelocity (lon/lat): {self.config.planning.v_initial}"
        for line in init_state_msg.split("\n"):
            util_logger.print_and_log_info(logger, line)

        util_logger.print_and_log_info(logger, "* Computing reachable sets...", verbose)

        time_start = time.time()
        self._reach.compute(step_start, step_end)
        self._reachable_set_computed = True
        time_computation = time.time() - time_start

        util_logger.print_and_log_info(logger, f"\tTook: \t{time_computation:.3f}s", verbose)

        # Save config to output folder
        if self.config.debug.save_config:
            path_output = self.config.general.path_output
            Path(path_output).mkdir(parents=True, exist_ok=True)
            self.config.save(path_output, str(self.config.scenario.scenario_id))
            util_logger.print_and_log_debug(logger, "\tConfiguration file saved.", verbose=True)

    def compute_drivable_area_at_step(self, step: int = 0):
        """
        Computes reachable sets between the given start and end steps.
        """
        if not self._reach:
            util_logger.print_and_log_warning(logger, "Reachable set is not initialized, aborting computation.")
            return None

        if step <= 0:
            util_logger.print_and_log_warning(logger, "Steps for computation are invalid, aborting computation.")
            return None

        self._reach.compute_drivable_area_at_step(step)
        self._reachable_set_computed = True

    def compute_reachable_set_at_step(self, step: int = 0):
        """
        Computes reachable sets between the given start and end steps.
        """
        if not self._reach:
            util_logger.print_and_log_warning(logger, "Reachable set is not initialized, aborting computation.")
            return None

        if step <= 0:
            util_logger.print_and_log_warning(logger, "Steps for computation are invalid, aborting computation.")
            return None

        self._reach.compute_drivable_area_at_step(step)

    def prune_reachable_sets(self):
        """
        Prunes reachable sets not reaching the final step.
        """
        self._reach.prune_nodes_not_reaching_final_step()

    def extract_driving_corridors(self, to_goal_region: bool = False, shape_terminal: Shape = None,
                                  is_cartesian_shape: bool = True, corridor_lon: DrivingCorridor = None,
                                  list_p_lon: List[float] = None) -> List[DrivingCorridor]:
        """
        Extracts driving corridors within the reachable sets.

        :param to_goal_region: whether the corridors should intersect with the goal region
        :param shape_terminal: a user-specified shape representing the terminal state
        :param is_cartesian_shape: whether the shape is given in Cartesian coordinate system
        :param corridor_lon: a longitudinal driving corridor
        :param list_p_lon: a list of longitudinal positions
        :return: a list of driving corridors
        """
        if not self.reachable_set:
            util_logger.print_and_log_warning(logger, "Reachable sets are empty! "
                                                      "Compute reachable sets before extracting driving corridors.")
            return []

        if not self._driving_corridor_extractor:
            self._driving_corridor_extractor = DrivingCorridorExtractor(self.reachable_set, self.config)

        util_logger.print_and_log_info(logger, f"* Extracting driving corridors...")
        time_start = time.time()
        list_corridors = self._driving_corridor_extractor.extract(to_goal_region, shape_terminal, is_cartesian_shape,
                                                                  corridor_lon, list_p_lon)
        time_computation = time.time() - time_start
        util_logger.print_and_log_info(logger, f"\tTook: \t{time_computation:.3f}s")

        return list_corridors
