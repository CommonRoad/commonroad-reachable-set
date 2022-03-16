import logging
import math

logger = logging.getLogger(__name__)

import numpy as np
from typing import Optional, Union

import pycrreachset as reach
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad_route_planner.route_planner import RoutePlanner
from omegaconf import ListConfig, DictConfig

from commonroad_reach.utility import configuration as util_configuration


class Configuration:
    """Class holding all relevant configurations"""

    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.name_scenario = config.general.name_scenario
        self.scenario: Optional[Scenario] = None
        self.planning_problem: Optional[PlanningProblem] = None

        self.general: GeneralConfiguration = GeneralConfiguration(config)
        self.vehicle: VehicleConfiguration = VehicleConfiguration(config)
        self.planning: PlanningConfiguration = PlanningConfiguration(config)
        self.reachable_set: ReachableSetConfiguration = ReachableSetConfiguration(config)
        self.debug: DebugConfiguration = DebugConfiguration(config)

    def complete_configuration(self, scenario: Scenario, planning_problem: PlanningProblem):
        self.scenario = scenario
        self.planning_problem = planning_problem

        self.vehicle.id_type_vehicle = planning_problem.planning_problem_id
        self.planning.complete_configuration(self)

    def print_configuration_summary(self):
        if self.planning.coordinate_system == "CART":
            CLCS = "Cartesian"
        elif self.planning.coordinate_system == "CVLN":
            CLCS = "Curvilinear"
        else:
            CLCS = "Undefined"

        if self.reachable_set.mode == 1:
            mode = "Single-step, Python backend with Python collision checker"
        elif self.reachable_set.mode == 2:
            mode = "Single-step, Python backend with C++ collision checker"
        elif self.reachable_set.mode == 3:
            mode = "Single-step, C++ backend with C++ collision checker"
        elif self.reachable_set.mode == 4:
            mode = "Graph-based (online), Python backend with C++ collision checker"
        elif self.reachable_set.mode == 5:
            mode = "Graph-based (offline), Python backend"
        else:
            mode = "Undefined"

        string = "# ===== Configuration Summary ===== #\n"
        string += f"# {self.scenario.scenario_id}\n"
        string += "# Planning:\n"
        string += f"# \ttime steps: {self.planning.time_steps_computation}, coordinate system: {CLCS}\n"
        string += "# Reachable set:\n"
        string += f"# \tmode: {mode}\n"
        string += f"# \tprune: {self.reachable_set.prune_nodes_not_reaching_final_time_step}\n"
        string += "# ================================= #"

        print(string)
        for line in string.split("\n"):
            logger.info(line)

    def convert_to_cpp_configuration(self) -> reach.Configuration:
        """Converts to configuration defined by the cpp side"""
        config = reach.Configuration()

        config.general.name_scenario = self.name_scenario
        config.general.path_scenarios = self.general.path_scenarios

        config.vehicle.ego.id_type_vehicle = self.vehicle.ego.id_type_vehicle
        config.vehicle.ego.length = self.vehicle.ego.length
        config.vehicle.ego.width = self.vehicle.ego.width
        config.vehicle.ego.v_lon_min = self.vehicle.ego.v_lon_min
        config.vehicle.ego.v_lon_max = self.vehicle.ego.v_lon_max
        config.vehicle.ego.v_lat_min = self.vehicle.ego.v_lat_min
        config.vehicle.ego.v_lat_max = self.vehicle.ego.v_lat_max
        config.vehicle.ego.a_lon_min = self.vehicle.ego.a_lon_min
        config.vehicle.ego.a_lon_max = self.vehicle.ego.a_lon_max
        config.vehicle.ego.a_lat_min = self.vehicle.ego.a_lat_min
        config.vehicle.ego.a_lat_max = self.vehicle.ego.a_lat_max
        config.vehicle.ego.a_max = self.vehicle.ego.a_max
        config.vehicle.ego.radius_disc = self.vehicle.ego.radius_disc
        config.vehicle.ego.wheelbase = self.vehicle.ego.wheelbase

        config.vehicle.other.id_type_vehicle = self.vehicle.other.id_type_vehicle
        config.vehicle.other.length = self.vehicle.other.length
        config.vehicle.other.width = self.vehicle.other.width
        config.vehicle.other.v_lon_min = self.vehicle.other.v_lon_min
        config.vehicle.other.v_lon_max = self.vehicle.other.v_lon_max
        config.vehicle.other.v_lat_min = self.vehicle.other.v_lat_min
        config.vehicle.other.v_lat_max = self.vehicle.other.v_lat_max
        config.vehicle.other.a_lon_min = self.vehicle.other.a_lon_min
        config.vehicle.other.a_lon_max = self.vehicle.other.a_lon_max
        config.vehicle.other.a_lat_min = self.vehicle.other.a_lat_min
        config.vehicle.other.a_lat_max = self.vehicle.other.a_lat_max
        config.vehicle.other.a_max = self.vehicle.other.a_max
        config.vehicle.other.radius_disc = self.vehicle.other.radius_disc
        config.vehicle.other.wheelbase = self.vehicle.other.wheelbase

        config.planning.dt = self.planning.dt
        config.planning.time_step_start = self.planning.time_step_start
        config.planning.time_steps_computation = self.planning.time_steps_computation
        config.planning.p_lon_initial = self.planning.p_lon_initial
        config.planning.p_lat_initial = self.planning.p_lat_initial
        config.planning.uncertainty_p_lon = self.planning.uncertainty_p_lon
        config.planning.uncertainty_p_lat = self.planning.uncertainty_p_lat
        config.planning.v_lon_initial = self.planning.v_lon_initial
        config.planning.v_lat_initial = self.planning.v_lat_initial
        config.planning.uncertainty_v_lon = self.planning.uncertainty_v_lon
        config.planning.uncertainty_v_lat = self.planning.uncertainty_v_lat
        config.planning.time_step_start = self.planning.time_step_initial
        config.planning.id_lanelet_initial = self.planning.id_lanelet_initial

        if self.planning.coordinate_system == "CART":
            config.planning.coordinate_system = reach.CoordinateSystem.CARTESIAN
        else:
            config.planning.coordinate_system = reach.CoordinateSystem.CURVILINEAR

        if self.planning.reference_point == "REAR":
            config.planning.reference_point = reach.ReferencePoint.REAR
        else:
            config.planning.reference_point = reach.ReferencePoint.CENTER

        config.reachable_set.size_grid = self.reachable_set.size_grid
        config.reachable_set.size_grid_2nd = self.reachable_set.size_grid_2nd
        config.reachable_set.radius_terminal_split = self.reachable_set.radius_terminal_split
        config.reachable_set.num_threads = self.reachable_set.num_threads

        return config


class GeneralConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.general
        name_scenario = config_relevant.name_scenario

        self.path_scenarios = config_relevant.path_scenarios
        self.path_scenario = config_relevant.path_scenarios + name_scenario + ".xml"
        self.path_output = config_relevant.path_output + name_scenario + "/"
        self.path_logs = config_relevant.path_logs
        self.path_offline_data = config_relevant.path_offline_data


class VehicleConfiguration:
    class Ego:
        def __init__(self, config: Union[ListConfig, DictConfig]):
            config_relevant = config.vehicle.ego

            self.id_type_vehicle = config_relevant.id_type_vehicle
            self.id_vehicle = None

            self.length = config_relevant.length
            self.width = config_relevant.width

            self.v_lon_min = config_relevant.v_lon_min
            self.v_lon_max = config_relevant.v_lon_max
            self.v_lat_min = config_relevant.v_lat_min
            self.v_lat_max = config_relevant.v_lat_max
            self.v_max = config_relevant.v_max

            self.a_lon_max = config_relevant.a_lon_max
            self.a_lon_min = config_relevant.a_lon_min
            self.a_lat_max = config_relevant.a_lat_max
            self.a_lat_min = config_relevant.a_lat_min
            self.a_max = config_relevant.a_max

            self.radius_disc, self.wheelbase = \
                util_configuration.compute_disc_radius_and_wheelbase(self.length, self.width)

    class Other:
        def __init__(self, config: Union[ListConfig, DictConfig]):
            config_relevant = config.vehicle.other

            self.id_type_vehicle = config_relevant.id_type_vehicle
            self.id_vehicle = None

            self.length = config_relevant.length
            self.width = config_relevant.width

            self.v_lon_min = config_relevant.v_lon_min
            self.v_lon_max = config_relevant.v_lon_max
            self.v_lat_min = config_relevant.v_lat_min
            self.v_lat_max = config_relevant.v_lat_max

            self.a_lon_max = config_relevant.a_lon_max
            self.a_lon_min = config_relevant.a_lon_min
            self.a_lat_max = config_relevant.a_lat_max
            self.a_lat_min = config_relevant.a_lat_min
            self.a_max = config_relevant.a_max

            self.radius_disc, self.wheelbase = \
                util_configuration.compute_disc_radius_and_wheelbase(self.length, self.width)

    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.ego = VehicleConfiguration.Ego(config)
        self.other = VehicleConfiguration.Other(config)


class PlanningConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.planning

        self.dt = config_relevant.dt
        self.time_step_start = config_relevant.time_step_start
        self.time_steps_computation = config_relevant.time_steps_computation

        self.p_lon_initial = None
        self.p_lat_initial = None
        self.uncertainty_p_lon = config_relevant.uncertainty_p_lon
        self.uncertainty_p_lat = config_relevant.uncertainty_p_lat

        self.v_lon_initial = None
        self.v_lat_initial = None
        self.uncertainty_v_lon = config_relevant.uncertainty_v_lon
        self.uncertainty_v_lat = config_relevant.uncertainty_v_lat

        self.o_initial = 0

        # related to specific planning problem
        self.time_step_initial = None
        self.id_lanelet_initial = 0
        self.route = None
        self.reference_path = None
        self.lanelet_network = None

        self.coordinate_system = config_relevant.coordinate_system
        self.reference_point = config_relevant.reference_point
        self.CLCS = None

    def complete_configuration(self, config: Configuration):
        scenario = config.scenario
        planning_problem = config.planning_problem

        self.lanelet_network = scenario.lanelet_network
        self.time_step_initial = planning_problem.initial_state.time_step

        if self.coordinate_system == "CART":
            p_initial, v_initial, o_initial = util_configuration.compute_initial_state_cart(config)

            self.p_lon_initial, self.p_lat_initial = p_initial
            self.v_lon_initial, self.v_lat_initial = v_initial
            self.o_initial = o_initial

        elif self.coordinate_system == "CVLN":
            # plans a route from the initial lanelet to the goal lanelet
            route_planner = RoutePlanner(scenario, planning_problem)
            candidate_holder = route_planner.plan_routes()
            route = candidate_holder.retrieve_first_route()

            self.route = route
            self.reference_path = route.reference_path
            self.id_lanelet_initial = route.list_ids_lanelets[0]

            self.CLCS = util_configuration.create_curvilinear_coordinate_system(self.reference_path)
            p_initial, v_initial = util_configuration.compute_initial_state_cvln(config)

            self.p_lon_initial, self.p_lat_initial = p_initial
            self.v_lon_initial, self.v_lat_initial = v_initial
    
    @property
    def p_lon_lat_initial(self):
        return np.array([self.p_lon_initial, self.p_lat_initial])

    @property
    def v_lon_lat_initial(self):
        return np.array([self.v_lon_initial, self.v_lat_initial])


class ReachableSetConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.reachable_set

        self.mode = config_relevant.mode
        self.num_threads = config_relevant.num_threads
        self.size_grid = config_relevant.size_grid
        self.size_grid_2nd = config_relevant.size_grid_2nd
        self.radius_terminal_split = config_relevant.radius_terminal_split
        self.prune_nodes_not_reaching_final_time_step = config_relevant.prune_nodes_not_reaching_final_time_step

        self.consider_traffic = config_relevant.consider_traffic
        self.name_pickle_offline = config_relevant.name_pickle_offline
        self.n_multi_steps = config_relevant.n_multi_steps


class DebugConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.debug

        self.save_plots = config_relevant.save_plots
        self.save_config = config_relevant.save_config
