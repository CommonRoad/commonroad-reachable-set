import os
import logging
from typing import Optional, Union, Tuple, List
from omegaconf import ListConfig, DictConfig

import numpy as np
import commonroad_reach.pycrreach as reach
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.common.solution import VehicleType
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
from commonroad_dc.geometry.util import resample_polyline
from commonroad_route_planner.route_planner import RoutePlanner

import commonroad_reach.utility.logger as util_logger
from commonroad_reach.utility import configuration as util_configuration
from commonroad_reach.utility import general as util_general

logger = logging.getLogger(__name__)


class Configuration:
    """
    Class holding all relevant configurations.
    """

    def __init__(self, config_omega: Union[ListConfig, DictConfig]):
        self.name_scenario = config_omega.general.name_scenario
        self.scenario: Optional[Scenario] = None
        self.planning_problem: Optional[PlanningProblem] = None

        self.general: GeneralConfiguration = GeneralConfiguration(config_omega)
        self.planning: PlanningConfiguration = PlanningConfiguration(config_omega)
        self.vehicle: VehicleConfiguration = VehicleConfiguration(config_omega)
        self.reachable_set: ReachableSetConfiguration = ReachableSetConfiguration(config_omega)
        self.debug: DebugConfiguration = DebugConfiguration(config_omega)

    def update_configuration(self, scenario: Scenario = None,
                             planning_problem: PlanningProblem = None, idx_planning_problem: int = 0,
                             state_initial: State = None, goal_region: GoalRegion = None,
                             CLCS: CurvilinearCoordinateSystem = None, list_ids_lanelets: List[int] = None):
        """
        Updates configuration based on the given attributes.

        Possible ways of completing the configuration:

            1. Empty attributes: loads scenario and planning problem from the xml files, computes route and CLCS
            2. Scenario + planning problem (+ initial state): computes route and CLCS
            3. Scenario + initial state + goal region: computes route and CLCS
            4. Scenario + initial state + CLCS: retrieve reference path from CLCS

        Additionally, a list of lanelet IDs (e.g., from a given route) can be passed to restrict the reachable set
        computation a-priori to a desired set of lanelets.
        """
        # patterns that do not require loading scenario and planning problem from xml files.
        if scenario and (planning_problem or
                         (state_initial and goal_region) or
                         (state_initial and CLCS)):
            pass

        else:
            scenario, planning_problem = util_general.load_scenario_and_planning_problem(self, idx_planning_problem)

        self.scenario = scenario
        self.planning_problem = planning_problem

        self.planning.state_initial = state_initial
        self.planning.goal_region = goal_region
        self.planning.CLCS = CLCS
        self.planning.list_ids_lanelets = list_ids_lanelets
        self.planning.update_configuration(self)

        self.vehicle.update_configuration(self)

        self.reachable_set.update_configuration(self)

    def print_configuration_summary(self):
        """
        Prints a summary of the configuration.
        """
        dict_clcs_to_string = {"CART": "cartesian", "CVLN": "curvilinear"}
        dict_mode_computation_to_string = {1: "polytopic, python backend", 2: "polytopic, c++ backend",
                                           3: "graph-based (online)", 4: "graph-based (offline)"}
        dict_mode_repartition_to_string = {1: "repartition, collision check", 2: "collision check, repartition",
                                           3: "repartition, collision check, then repartition"}
        dict_mode_inflation_to_string = {1: "inscribed circle", 2: "circumscribed circle",
                                         3: "three circle approximation"}

        CLCS = dict_clcs_to_string[self.planning.coordinate_system]
        mode_computation = dict_mode_computation_to_string[self.reachable_set.mode_computation]
        mode_repartition = dict_mode_repartition_to_string[self.reachable_set.mode_repartition]
        mode_inflation = dict_mode_inflation_to_string[self.reachable_set.mode_inflation]

        string = "# ===== Configuration Summary ===== #\n"
        string += f"# {self.scenario.scenario_id}\n"
        string += "# Planning:\n"
        string += f"# \tdt: {self.planning.dt}\n"
        string += f"# \tsteps: {self.planning.steps_computation}\n"
        string += f"# \tcoordinate system: {CLCS}\n"

        config_ego = self.vehicle.ego
        string += "# Vehicle (Ego):\n"
        string += f"# \tvehicle type id: {self.vehicle.ego.id_type_vehicle}\n"
        string += f"# \tv: lon_min = {config_ego.v_lon_min}, lon_max = {config_ego.v_lon_max}, " \
                  f"lat_min = {config_ego.v_lat_min}, lat_max = {config_ego.v_lat_max}, max = {config_ego.v_max}\n"
        string += f"# \ta: lon_min = {config_ego.a_lon_min}, lon_max = {config_ego.a_lon_max}, " \
                  f"lat_min = {config_ego.a_lat_min}, lat_max = {config_ego.a_lat_max}, max = {config_ego.a_max}\n"

        string += "# Reachable set:\n"
        string += f"# \tcomputation mode: {mode_computation}\n"
        string += f"# \trepartition mode: {mode_repartition}\n"
        string += f"# \tinflation mode: {mode_inflation}\n"
        string += f"# \tgrid size: {self.reachable_set.size_grid}\n"
        string += f"# \tsplit radius: {self.reachable_set.radius_terminal_split}\n"
        string += f"# \tprune: {self.reachable_set.prune_nodes_not_reaching_final_step}\n"
        string += f"# \tnum threads: {self.reachable_set.num_threads}\n"
        string += "# ================================= #"

        for line in string.split("\n"):
            util_logger.print_and_log_info(logger, line)

    def convert_to_cpp_configuration(self) -> reach.Configuration:
        """
        Converts to a configuration that is readable by the C++ binding code.
        """
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
        config.vehicle.ego.circle_distance = self.vehicle.ego.circle_distance
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
        config.vehicle.other.circle_distance = self.vehicle.other.circle_distance
        config.vehicle.other.wheelbase = self.vehicle.other.wheelbase

        config.planning.dt = self.planning.dt
        config.planning.step_start = self.planning.step_start
        config.planning.steps_computation = self.planning.steps_computation
        config.planning.p_lon_initial = self.planning.p_lon_initial
        config.planning.p_lat_initial = self.planning.p_lat_initial
        config.planning.uncertainty_p_lon = self.planning.uncertainty_p_lon
        config.planning.uncertainty_p_lat = self.planning.uncertainty_p_lat
        config.planning.v_lon_initial = self.planning.v_lon_initial
        config.planning.v_lat_initial = self.planning.v_lat_initial
        config.planning.uncertainty_v_lon = self.planning.uncertainty_v_lon
        config.planning.uncertainty_v_lat = self.planning.uncertainty_v_lat

        if self.planning.coordinate_system == "CART":
            config.planning.coordinate_system = reach.CoordinateSystem.CARTESIAN

        else:
            config.planning.coordinate_system = reach.CoordinateSystem.CURVILINEAR
            config.planning.CLCS = self.planning.CLCS

        if self.planning.reference_point == "REAR":
            config.planning.reference_point = reach.ReferencePoint.REAR

        else:
            config.planning.reference_point = reach.ReferencePoint.CENTER

        config.reachable_set.mode_repartition = self.reachable_set.mode_repartition
        config.reachable_set.mode_inflation = self.reachable_set.mode_inflation
        config.reachable_set.size_grid = self.reachable_set.size_grid
        config.reachable_set.size_grid_2nd = self.reachable_set.size_grid_2nd
        config.reachable_set.radius_terminal_split = self.reachable_set.radius_terminal_split
        config.reachable_set.num_threads = self.reachable_set.num_threads
        config.reachable_set.prune_nodes = self.reachable_set.prune_nodes_not_reaching_final_step
        config.reachable_set.rasterize_obstacles = self.reachable_set.rasterize_obstacles

        # convert lut dict to Cpp configuration via PyBind function
        if self.reachable_set.mode_inflation == 3:
            config.reachable_set.lut_lon_enlargement = \
                reach.LUTLongitudinalEnlargement(self.reachable_set.lut_longitudinal_enlargement)

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

            # load vehicle parameters according to id_type_vehicle
            try:
                vehicle_parameters = VehicleParameterMapping.from_vehicle_type(VehicleType(self.id_type_vehicle))

            except KeyError:
                raise Exception(f"Given vehicle type id {self.id_type_vehicle} is not valid.")

            self.length = vehicle_parameters.l
            self.width = vehicle_parameters.w

            self.v_lon_min = vehicle_parameters.longitudinal.v_min
            self.v_lon_max = vehicle_parameters.longitudinal.v_max
            # not present in vehicle parameters
            self.v_lat_min = None
            self.v_lat_max = None
            self.v_max = vehicle_parameters.longitudinal.v_max

            self.a_lon_min = -vehicle_parameters.longitudinal.a_max
            self.a_lon_max = vehicle_parameters.longitudinal.a_max
            # not present in vehicle parameters
            self.a_lat_min = None
            self.a_lat_max = None
            self.a_max = vehicle_parameters.longitudinal.a_max

            # distances front/rear axle to vehicle center
            self.wb_front_axle = vehicle_parameters.a
            self.wb_rear_axle = vehicle_parameters.b
            # wheelbase
            self.wheelbase = self.wb_front_axle + self.wb_rear_axle

            # overwrite with parameters given by vehicle ID if they are explicitly provided in the *.yaml file
            for key, value in config_relevant.items():
                if value is not None:
                    setattr(self, key, value)

            self.radius_disc, self.circle_distance = \
                util_configuration.compute_disc_radius_and_distance(self.length, self.width,
                                                                    ref_point=config.planning.reference_point,
                                                                    dist_axle_rear=self.wb_rear_axle)

            self.radius_inflation = util_configuration.compute_inflation_radius(config.reachable_set.mode_inflation,
                                                                                self.length, self.width,
                                                                                self.radius_disc)
            self.update_configuration(config)

        def update_configuration(self, config):
            # overwrite velocity and acceleration parameters if computing within Cartesian coordinate system
            if config.planning.coordinate_system == "CART":
                self.v_lon_min = self.v_lat_min = -self.v_max
                self.v_lon_max = self.v_lat_max = self.v_max
                self.a_lon_min = self.a_lat_min = -self.a_max
                self.a_lon_max = self.a_lat_max = self.a_max

    class Other:
        def __init__(self, config: Union[ListConfig, DictConfig]):
            config_relevant = config.vehicle.other

            self.id_type_vehicle = config_relevant.id_type_vehicle
            # load vehicle parameters according to id_type_vehicle
            try:
                vehicle_parameters = VehicleParameterMapping.from_vehicle_type(VehicleType(self.id_type_vehicle))

            except KeyError:
                raise Exception(f"Given vehicle type id {self.id_type_vehicle} is not valid.")

            self.length = vehicle_parameters.l
            self.width = vehicle_parameters.w

            self.v_lon_min = vehicle_parameters.longitudinal.v_min
            self.v_lon_max = vehicle_parameters.longitudinal.v_max
            # not present in vehicle parameters
            self.v_lat_min = None
            self.v_lat_max = None
            self.v_max = vehicle_parameters.longitudinal.v_max

            self.a_lon_min = -vehicle_parameters.longitudinal.a_max
            self.a_lon_max = vehicle_parameters.longitudinal.a_max
            # not present in vehicle parameters
            self.a_lat_min = None
            self.a_lat_max = None
            self.a_max = vehicle_parameters.longitudinal.a_max

            # distances front/rear axle to vehicle center
            self.wb_front_axle = vehicle_parameters.a
            self.wb_rear_axle = vehicle_parameters.b
            # wheelbase
            self.wheelbase = self.wb_front_axle + self.wb_rear_axle

            # overwrite with parameters in the config file
            for key, value in config_relevant.items():
                if value is not None:
                    setattr(self, key, value)

            self.radius_disc, self.circle_distance = \
                util_configuration.compute_disc_radius_and_distance(self.length, self.width,
                                                                    ref_point=config.planning.reference_point,
                                                                    dist_axle_rear=self.wb_rear_axle)

            self.radius_inflation = util_configuration.compute_inflation_radius(config.reachable_set.mode_inflation,
                                                                                self.length, self.width,
                                                                                self.radius_disc)

    def __init__(self, config: Union[ListConfig, DictConfig]):
        self.ego = VehicleConfiguration.Ego(config)
        self.other = VehicleConfiguration.Other(config)

    def update_configuration(self, config: Configuration):
        self.ego.update_configuration(config)


class PlanningConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.planning

        self.dt = config_relevant.dt
        self.step_start = config_relevant.step_start
        self.steps_computation = config_relevant.steps_computation

        self.p_lon_initial = None
        self.p_lat_initial = None
        self.uncertainty_p_lon = config_relevant.uncertainty_p_lon
        self.uncertainty_p_lat = config_relevant.uncertainty_p_lat
        self.v_lon_initial = None
        self.v_lat_initial = None
        self.uncertainty_v_lon = config_relevant.uncertainty_v_lon
        self.uncertainty_v_lat = config_relevant.uncertainty_v_lat
        self.o_initial = None

        # related to specific planning problem
        self.state_initial = None
        self.goal_region = None
        self.reference_path = None
        self.lanelet_network = None
        self.list_ids_lanelets = None
        self.CLCS = None
        self.coordinate_system = config_relevant.coordinate_system
        self.reference_point = config_relevant.reference_point

    def update_configuration(self, config: Configuration):
        scenario = config.scenario
        planning_problem = config.planning_problem

        self.lanelet_network = scenario.lanelet_network if not self.list_ids_lanelets \
            else util_general.create_lanelet_network_from_ids(scenario.lanelet_network, self.list_ids_lanelets)

        self.step_start = planning_problem.initial_state.time_step \
            if not self.state_initial else self.state_initial.time_step

        assert round(self.dt * 100) % round(scenario.dt * 100) == 0, \
            f"Value of dt ({self.dt}) should be a multiple of scenario dt ({scenario.dt})."

        if self.coordinate_system == "CART":
            p_initial, v_initial, o_initial = util_configuration.compute_initial_state_cart(config)

            self.p_lon_initial, self.p_lat_initial = p_initial
            self.v_lon_initial, self.v_lat_initial = v_initial
            self.o_initial = o_initial

        elif self.coordinate_system == "CVLN":
            if self.CLCS:
                # CLCS is given, retrieve reference path
                self.reference_path = np.array(self.CLCS.reference_path())

            else:
                # plans a route from the initial lanelet to the goal lanelet, set curvilinear coordinate system
                route_planner = RoutePlanner(lanelet_network=scenario.lanelet_network,
                                             planning_problem=planning_problem,
                                             state_initial=self.state_initial,
                                             goal_region=self.goal_region)
                candidate_holder = route_planner.plan_routes()
                route = candidate_holder.retrieve_first_route()

                if route:
                    ref_path_mod = resample_polyline(route.reference_path, 0.5)
                    self.reference_path = ref_path_mod
                    self.CLCS = util_configuration.create_curvilinear_coordinate_system(self.reference_path)
                    self.reference_path = np.array(self.CLCS.reference_path())

            p_initial, v_initial = util_configuration.compute_initial_state_cvln(config, self.state_initial)

            self.p_lon_initial, self.p_lat_initial = p_initial
            self.v_lon_initial, self.v_lat_initial = v_initial

    @property
    def p_initial(self):
        return np.array([self.p_lon_initial, self.p_lat_initial])

    @p_initial.setter
    def p_initial(self, p_initial: Tuple):
        assert (type(p_initial is tuple)), "Initial lon/lat position must be of type tuple with length 2"
        assert (len(p_initial) == 2), "Initial lon/lat position must be of type tuple with length 2"

        self.p_lon_initial = p_initial[0]
        self.p_lat_initial = p_initial[1]

    @property
    def v_initial(self):
        return np.array([self.v_lon_initial, self.v_lat_initial])

    @v_initial.setter
    def v_initial(self, v_initial: Tuple):
        assert (type(v_initial is tuple)), "Initial lon/lat velocity must be of type tuple with length 2"
        assert (len(v_initial) == 2), "Initial lon/lat velocity must be of type tuple with length 2"

        self.v_lon_initial = v_initial[0]
        self.v_lat_initial = v_initial[1]


class ReachableSetConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.reachable_set

        self.mode_computation = config_relevant.mode_computation
        self.mode_repartition = config_relevant.mode_repartition
        self.mode_inflation = config_relevant.mode_inflation
        self.consider_traffic = config_relevant.consider_traffic
        self.rasterize_obstacles = config_relevant.rasterize_obstacles

        self.size_grid = config_relevant.size_grid
        self.size_grid_2nd = config_relevant.size_grid_2nd
        self.radius_terminal_split = config_relevant.radius_terminal_split
        self.prune_nodes_not_reaching_final_step = config_relevant.prune_nodes_not_reaching_final_step

        self.name_pickle_offline = config_relevant.name_pickle_offline
        self.n_multi_steps = config_relevant.n_multi_steps

        self.num_threads = config_relevant.num_threads

        self.path_to_lut = os.path.abspath(
            os.path.join(config.general.path_scenarios, "..", config_relevant.path_to_lut))
        self.lut_longitudinal_enlargement = None

    def update_configuration(self, config: Configuration):
        if self.mode_inflation == 3:
            self.lut_longitudinal_enlargement = util_configuration.read_lut_longitudinal_enlargement(
                config.planning.reference_point, config.vehicle.ego.circle_distance, self.path_to_lut)


class DebugConfiguration:
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.debug

        self.save_plots = config_relevant.save_plots
        self.save_config = config_relevant.save_config
        self.verbose_debug = config_relevant.verbose_debug
        self.verbose_info = config_relevant.verbose_info
        self.draw_ref_path = config_relevant.draw_ref_path
        self.draw_planning_problem = config_relevant.draw_planning_problem
        self.draw_icons = config_relevant.draw_icons
        self.draw_lanelet_labels = config_relevant.draw_lanelet_labels
        self.plot_limits = config_relevant.plot_limits
        self.plot_azimuth = config_relevant.plot_azimuth
        self.plot_elevation = config_relevant.plot_elevation
        self.ax_distance = config_relevant.ax_distance
