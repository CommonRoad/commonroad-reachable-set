import os
import copy
import logging
from typing import Optional, Union, Tuple, List
import yaml

from omegaconf import ListConfig, DictConfig
import numpy as np

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.common.solution import VehicleType

from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping

from commonroad_clcs.pycrccosy import CurvilinearCoordinateSystem
from commonroad_clcs.util import resample_polyline

from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner

import commonroad_reach.utility.logger as util_logger
from commonroad_reach.utility import configuration as util_configuration
from commonroad_reach.utility import general as util_general
from commonroad_reach import pycrreach

logger = logging.getLogger("REACH_LOGGER")


class Configuration:
    """
    Class holding all relevant configurations.
    """

    def __init__(self, config_omega: Union[ListConfig, DictConfig]):
        self.config_omega = config_omega
        self.name_scenario = config_omega.general.name_scenario
        self.scenario: Optional[Scenario] = None
        self.planning_problem: Optional[PlanningProblem] = None
        self.planning_problem_set: Optional[PlanningProblemSet] = None

        self.general: GeneralConfiguration = GeneralConfiguration(config_omega)
        self.planning: PlanningConfiguration = PlanningConfiguration(config_omega)
        self.vehicle: VehicleConfiguration = VehicleConfiguration(config_omega)
        self.reachable_set: ReachableSetConfiguration = ReachableSetConfiguration(config_omega)
        self.debug: DebugConfiguration = DebugConfiguration(config_omega)

    def __repr__(self):
        return f"Configuration(scenario_id={self.scenario.scenario_id}," \
               f"planning_problem_id={self.planning_problem.planning_problem_id})"

    def update(self, scenario: Scenario = None, planning_problem_set: PlanningProblemSet = None,
               planning_problem: PlanningProblem = None, idx_planning_problem: int = 0,
               CLCS: CurvilinearCoordinateSystem = None, list_ids_lanelets: List[int] = None):
        """
        Updates configuration based on the given attributes.

        Possible ways of completing the configuration:

            1. Empty attributes: loads scenario and planning problem (set) from the xml files, computes route and CLCS
            2. Scenario + planning problem (set): retrieves initial state from planning problem,
                                                  computes reference path and CLCS
            3. Scenario + planning problem + CLCS: retrieves initial state from planning problem,
                                                   retrieves reference path from CLCS

        Additionally, a list of lanelet IDs (e.g., from a given route) can be passed to restrict the reachable set
        computation a-priori to a desired set of lanelets.
        """
        # patterns that do not require loading scenario and planning problem from xml files.
        if scenario and (planning_problem_set or
                         planning_problem or
                         (planning_problem and CLCS)):
            pass

        else:
            scenario, planning_problem_set = CommonRoadFileReader(self.general.path_scenario).open()

        self.scenario = scenario

        self.planning_problem_set = planning_problem_set

        if planning_problem_set and not planning_problem:
            planning_problem = list(planning_problem_set.planning_problem_dict.values())[idx_planning_problem]

        self.planning_problem = planning_problem

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

        string = "\n# ===== CommonRoad-Reach Configuration Summary ===== #\n"
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
        string += f"# \tobstacle rasterization: {self.reachable_set.rasterize_obstacles}\n"
        string += f"# \tgrid size: {self.reachable_set.size_grid}\n"
        string += f"# \tsplit radius: {self.reachable_set.radius_terminal_split}\n"
        string += f"# \tprune: {self.reachable_set.prune_nodes_not_reaching_final_step}\n"
        string += f"# \tnum threads: {self.reachable_set.num_threads}\n"
        string += "# ================================= #"

        for line in string.split("\n"):
            util_logger.print_and_log_info(logger, line)

    def save(self, path_save: str, name_file: str):
        """
        Saves configuration to a yaml file.
        """
        dict_save = {"name_scenario": self.name_scenario}

        for name_obj, obj in self.__dict__.items():
            try:
                dict_obj = obj.to_dict()

            except AttributeError:
                continue

            else:
                dict_save.update({name_obj: dict_obj})

        with open(f'{path_save}/{name_file}.yml', 'w') as file_yaml:
            yaml.dump(dict_save, file_yaml, default_flow_style=False, allow_unicode=True)

    def convert_to_cpp_configuration(self) -> pycrreach.Configuration:
        """
        Converts to a configuration that is readable by the C++ binding code.
        """
        config = pycrreach.Configuration()

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
            config.planning.coordinate_system = pycrreach.CoordinateSystem.CARTESIAN

        else:
            config.planning.coordinate_system = pycrreach.CoordinateSystem.CURVILINEAR
            config.planning.CLCS = self.planning.CLCS

        if self.planning.reference_point == "REAR":
            config.planning.reference_point = pycrreach.ReferencePoint.REAR

        else:
            config.planning.reference_point = pycrreach.ReferencePoint.CENTER

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
                pycrreach.LUTLongitudinalEnlargement(self.reachable_set.lut_longitudinal_enlargement)

        return config

    def clone(self):
        config_cloned = Configuration(self.config_omega)

        for key, val in self.__dict__.items():
            config_cloned.__dict__[key] = copy.deepcopy(val)

        return config_cloned

    def split_to_planning_problems(self):
        list_config = list()

        assert self.planning_problem_set, "Planning problem set is not provided."

        for idx_pp, pp in self.planning_problem_set.planning_problem_dict.items():
            config = self.clone()
            config.update(scenario=self.scenario, planning_problem=pp)
            list_config.append(config)

        return list_config


class ConfigurationBase:
    def to_dict(self):
        dict_config = dict()
        for key, val in self.__dict__.items():
            if isinstance(val, np.float64):
                val = float(val)

            if isinstance(val, (str, int, float, bool)):
                dict_config[key] = val

        return dict_config


class GeneralConfiguration(ConfigurationBase):
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.general
        name_scenario = config_relevant.name_scenario

        self.path_scenarios = config_relevant.path_scenarios
        self.path_scenario = config_relevant.path_scenarios + name_scenario + ".xml"
        self.path_output = config_relevant.path_output + name_scenario + "/"
        self.path_logs = config_relevant.path_logs
        self.path_offline_data = config_relevant.path_offline_data
        self.path_pickles = config_relevant.path_pickles


class VehicleConfiguration(ConfigurationBase):
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

            # overwrite with parameters given by vehicle ID if they are explicitly provided in the *.yaml file
            for key, value in config_relevant.items():
                if value is not None:
                    setattr(self, key, value)

            # wheelbase
            self.wheelbase = self.wb_front_axle + self.wb_rear_axle

            self.radius_disc, self.circle_distance = \
                util_configuration.compute_disc_radius_and_distance(self.length, self.width,
                                                                    ref_point=config.planning.reference_point,
                                                                    dist_axle_rear=self.wb_rear_axle)
            assert not (config.planning.reference_point == "REAR" and config.reachable_set.mode_inflation == 2), \
                "Circumscribed inflation only supports reference point CENTER"
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

    def to_dict(self):
        dict_config = {"ego": dict(), "other": dict()}
        for key, val in self.ego.__dict__.items():
            if isinstance(val, np.float64):
                val = float(val)

            if isinstance(val, (str, int, float, bool)):
                dict_config["ego"][key] = val

        for key, val in self.other.__dict__.items():
            if isinstance(val, np.float64):
                val = float(val)

            if isinstance(val, (str, int, float, bool)):
                dict_config["other"][key] = val

        return dict_config


class PlanningConfiguration(ConfigurationBase):
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
        self.route = None
        self.reference_path = None
        self.lanelet_network = None
        self.list_ids_lanelets = None
        self.CLCS = None
        self.coordinate_system = config_relevant.coordinate_system
        self.reference_point = config_relevant.reference_point

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

    def set_initial_states(
            self,
            step_initial: float,
            pos_initial: Tuple[float, float],
            vel_initial: Tuple[float, float],
            theta_initial: Optional[float] = None
        ) -> None:
        """
        Sets the following attributes of the initial state required for reach. set computation
        :param step_initial: initial time step
        :param pos_initial: initial position as Tuple of [p_lon, p_lat]
        :param vel_initial: initial velocity as Tuple of [v_lon, v_lat]
        :param theta_initial: initial orientation as float (Optional, only for Cartesian computation)
        """
        self.step_start = step_initial
        self.p_initial = pos_initial
        self.v_initial = vel_initial
        self.o_initial = theta_initial

    def update_configuration(self, config: Configuration):
        scenario = config.scenario
        planning_problem = config.planning_problem

        self.lanelet_network = scenario.lanelet_network if not self.list_ids_lanelets \
            else util_general.create_lanelet_network_from_ids(scenario.lanelet_network, self.list_ids_lanelets)

        step_start = planning_problem.initial_state.time_step

        assert round(self.dt * 100) % round(scenario.dt * 100) == 0, \
            f"Value of dt ({self.dt}) should be a multiple of scenario dt ({scenario.dt})."

        if self.coordinate_system == "CART":
            p_initial, v_initial, o_initial = util_configuration.compute_initial_state_cart(config)

            self.set_initial_states(step_start, p_initial, v_initial, o_initial)

            v_max = config.vehicle.ego.v_max
            assert -v_max <= self.v_lon_initial <= v_max, \
                f"Initial x velocity {self.v_lon_initial} exceeds valid velocity interval [{-v_max}, {v_max}]."

            assert -v_max <= self.v_lat_initial <= v_max, \
                f"Initial y velocity {self.v_lat_initial} exceeds valid velocity interval [{-v_max}, {v_max}]."

        elif self.coordinate_system == "CVLN":
            if self.CLCS:
                # CLCS is given, retrieve reference path
                self.reference_path = np.array(self.CLCS.reference_path())

            else:
                # TODO remove this logic from the configuration object
                # plans a route from the initial lanelet to the goal lanelet, set curvilinear coordinate system
                route_planner = RoutePlanner(
                    lanelet_network=scenario.lanelet_network,
                    planning_problem=planning_problem
                )
                routes = route_planner.plan_routes()

                if routes:
                    ref_path_planner = ReferencePathPlanner(
                        lanelet_network=scenario.lanelet_network,
                        planning_problem=planning_problem,
                        routes=routes
                    )
                    ref_path_object = ref_path_planner.plan_shortest_reference_path()
                    ref_path = ref_path_object.reference_path
                    self.reference_path = resample_polyline(ref_path, 0.5)
                    self.CLCS = util_configuration.create_curvilinear_coordinate_system(self.reference_path)
                    self.reference_path = np.array(self.CLCS.reference_path())

            p_initial, v_initial = util_configuration.compute_initial_state_cvln(config)

            self.set_initial_states(step_start, p_initial, v_initial)

            assert config.vehicle.ego.v_lon_min <= self.v_lon_initial <= config.vehicle.ego.v_lon_max, \
                f"Initial longitudinal velocity {self.v_lon_initial} exceeds valid velocity interval " \
                f"[{config.vehicle.ego.v_lon_min}, {config.vehicle.ego.v_lon_max}]."

            assert config.vehicle.ego.v_lat_min <= self.v_lat_initial <= config.vehicle.ego.v_lat_max, \
                f"Initial lateral velocity {self.v_lat_initial} exceeds valid velocity interval " \
                f"[{config.vehicle.ego.v_lat_min}, {config.vehicle.ego.v_lat_max}]."


class ReachableSetConfiguration(ConfigurationBase):
    def __init__(self, config: Union[ListConfig, DictConfig]):
        config_relevant = config.reachable_set

        self.mode_computation = config_relevant.mode_computation
        self.mode_repartition = config_relevant.mode_repartition
        self.mode_inflation = config_relevant.mode_inflation
        self.consider_traffic = config_relevant.consider_traffic
        self.rasterize_obstacles = config_relevant.rasterize_obstacles
        self.rasterize_exclude_static = config_relevant.rasterize_exclude_static

        self.size_grid = config_relevant.size_grid
        self.size_grid_2nd = config_relevant.size_grid_2nd
        self.radius_terminal_split = config_relevant.radius_terminal_split
        self.prune_nodes_not_reaching_final_step = config_relevant.prune_nodes_not_reaching_final_step
        self.exclude_small_components_corridor = config_relevant.exclude_small_components_corridor

        self.name_pickle_offline = config_relevant.name_pickle_offline
        self.n_multi_steps = config_relevant.n_multi_steps

        self.num_threads = config_relevant.num_threads

        self.path_to_lut = os.path.abspath(
            os.path.join(config.general.path_scenarios, "..", config_relevant.path_to_lut))
        self.lut_longitudinal_enlargement = None

    def update_configuration(self, config: Configuration):
        if self.mode_inflation == 3 and self.lut_longitudinal_enlargement is None:
            self.lut_longitudinal_enlargement = util_configuration.read_lut_longitudinal_enlargement(
                config.planning.reference_point, config.vehicle.ego.circle_distance, self.path_to_lut)


class DebugConfiguration(ConfigurationBase):
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
