from typing import Dict, Optional

# import pycrreachset as reach
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad_route_planner.route_planner import RoutePlanner

from commonroad_reachset.utility import configugation as util_configuration


class Configuration:
    """Class holding all relevant configurations"""

    def __init__(self, dict_config: Dict):
        self.name_scenario = dict_config["config_general"]["name_scenario"]
        self.scenario: Optional[Scenario] = None
        self.planning_problem: Optional[PlanningProblem] = None

        self.config_general = GeneralConfiguration(dict_config)
        self.config_vehicle = VehicleConfiguration(dict_config)
        self.config_planning = PlanningConfiguration(dict_config)
        self.config_reachable_set = ReachableSetConfiguration(dict_config)
        self.config_sonia = SONIAConfiguration(dict_config)
        self.config_debug = DebugConfiguration(dict_config)

    @property
    def general(self):
        return self.config_general

    @property
    def vehicle(self):
        return self.config_vehicle

    @property
    def planning(self):
        return self.config_planning

    @property
    def reachable_set(self):
        return self.config_reachable_set

    @property
    def sonia(self):
        return self.config_sonia

    @property
    def debug(self):
        return self.config_debug

    def complete_configuration(self, scenario, planning_problem):
        self.scenario = scenario
        self.planning_problem = planning_problem

        self.vehicle.id_type_vehicle = planning_problem.planning_problem_id
        self.planning.complete_configuration(self)

    # def convert_to_cpp_configuration(self) -> reach.Configuration:
    #     """Converts to configuration defined by the cpp side"""
    #     config = reach.Configuration()
    #
    #     config.general.name_scenario = self.name_scenario
    #     config.general.path_scenarios = self.general.path_scenarios
    #
    #     config.vehicle.ego.id_type_vehicle = self.vehicle.ego.id_type_vehicle
    #     config.vehicle.ego.length = self.vehicle.ego.length
    #     config.vehicle.ego.width = self.vehicle.ego.width
    #     config.vehicle.ego.v_lon_min = self.vehicle.ego.v_lon_min
    #     config.vehicle.ego.v_lon_max = self.vehicle.ego.v_lon_max
    #     config.vehicle.ego.v_lat_min = self.vehicle.ego.v_lat_min
    #     config.vehicle.ego.v_lat_max = self.vehicle.ego.v_lat_max
    #     config.vehicle.ego.a_lon_min = self.vehicle.ego.a_lon_min
    #     config.vehicle.ego.a_lon_max = self.vehicle.ego.a_lon_max
    #     config.vehicle.ego.a_lat_min = self.vehicle.ego.a_lat_min
    #     config.vehicle.ego.a_lat_max = self.vehicle.ego.a_lat_max
    #     config.vehicle.ego.a_max = self.vehicle.ego.a_max
    #     config.vehicle.ego.t_react = self.vehicle.ego.t_react
    #     config.vehicle.ego.radius_disc = self.vehicle.ego.radius_disc
    #     config.vehicle.ego.wheelbase = self.vehicle.ego.wheelbase
    #
    #     config.vehicle.other.id_type_vehicle = self.vehicle.other.id_type_vehicle
    #     config.vehicle.other.length = self.vehicle.other.length
    #     config.vehicle.other.width = self.vehicle.other.width
    #     config.vehicle.other.v_lon_min = self.vehicle.other.v_lon_min
    #     config.vehicle.other.v_lon_max = self.vehicle.other.v_lon_max
    #     config.vehicle.other.v_lat_min = self.vehicle.other.v_lat_min
    #     config.vehicle.other.v_lat_max = self.vehicle.other.v_lat_max
    #     config.vehicle.other.a_lon_min = self.vehicle.other.a_lon_min
    #     config.vehicle.other.a_lon_max = self.vehicle.other.a_lon_max
    #     config.vehicle.other.a_lat_min = self.vehicle.other.a_lat_min
    #     config.vehicle.other.a_lat_max = self.vehicle.other.a_lat_max
    #     config.vehicle.other.a_max = self.vehicle.other.a_max
    #     config.vehicle.other.t_react = self.vehicle.other.t_react
    #     config.vehicle.other.radius_disc = self.vehicle.other.radius_disc
    #     config.vehicle.other.wheelbase = self.vehicle.other.wheelbase
    #
    #     config.planning.dt = self.planning.dt
    #     config.planning.time_step_start = self.planning.time_step_start
    #     config.planning.time_steps_computation = self.planning.time_steps_computation
    #     config.planning.p_lon_initial = self.planning.p_lon_initial
    #     config.planning.p_lat_initial = self.planning.p_lat_initial
    #     config.planning.uncertainty_p_lon = self.planning.uncertainty_p_lon
    #     config.planning.uncertainty_p_lat = self.planning.uncertainty_p_lat
    #     config.planning.v_lon_initial = self.planning.v_lon_initial
    #     config.planning.v_lat_initial = self.planning.v_lat_initial
    #     config.planning.uncertainty_v_lon = self.planning.uncertainty_v_lon
    #     config.planning.uncertainty_v_lat = self.planning.uncertainty_v_lat
    #     config.planning.time_step_start = self.planning.time_step_initial
    #     config.planning.id_lanelet_initial = self.planning.id_lanelet_initial
    #
    #     if self.planning.coordinate_system == "CART":
    #         config.planning.coordinate_system = reach.CoordinateSystem.CARTESIAN
    #     else:
    #         config.planning.coordinate_system = reach.CoordinateSystem.CURVILINEAR
    #
    #     if self.planning.reference_point == "REAR":
    #         config.planning.reference_point = reach.ReferencePoint.REAR
    #     else:
    #         config.planning.reference_point = reach.ReferencePoint.CENTER
    #
    #     config.reachable_set.size_grid = self.reachable_set.size_grid
    #     config.reachable_set.size_grid_2nd = self.reachable_set.size_grid_2nd
    #     config.reachable_set.radius_terminal_split = self.reachable_set.radius_terminal_split
    #     config.reachable_set.num_threads = self.reachable_set.num_threads
    #
    #     config.debug.verbose_mode = self.debug.verbose_mode
    #     config.debug.measure_time = self.debug.measure_time
    #
    #     return config


class GeneralConfiguration:
    def __init__(self, dict_config: Dict):
        dict_relevant = dict_config["config_general"]
        name_scenario = dict_relevant["name_scenario"]

        self.path_scenarios = dict_relevant["path_scenarios"]
        self.path_scenario = dict_relevant["path_scenarios"] + name_scenario + ".xml"
        self.path_output = dict_relevant["path_output"] + name_scenario + "/"


class VehicleConfiguration:
    class Ego:
        def __init__(self, dict_config: Dict):
            dict_relevant = dict_config["config_vehicle"]["ego"]

            self.id_type_vehicle = dict_relevant["id_type_vehicle"]
            self.id_vehicle = None

            self.length = dict_relevant["length"]
            self.width = dict_relevant["width"]

            self.v_lon_min = dict_relevant["v_lon_min"]
            self.v_lon_max = dict_relevant["v_lon_max"]
            self.v_lat_min = dict_relevant["v_lat_min"]
            self.v_lat_max = dict_relevant["v_lat_max"]

            self.a_lon_max = dict_relevant["a_lon_max"]
            self.a_lon_min = dict_relevant["a_lon_min"]
            self.a_lat_max = dict_relevant["a_lat_max"]
            self.a_lat_min = dict_relevant["a_lat_min"]
            self.a_max = dict_relevant["a_max"]

            self.j_lon_min = dict_relevant["j_lon_min"]
            self.j_lon_max = dict_relevant["j_lon_max"]
            self.j_lat_min = dict_relevant["j_lat_min"]
            self.j_lat_max = dict_relevant["j_lat_max"]

            self.t_react = dict_relevant["t_react"]

            self.fov = dict_relevant["fov"]

            self.radius_disc, self.wheelbase = \
                util_configuration.compute_disc_radius_and_wheelbase(self.length, self.width)

    class Other:
        def __init__(self, dict_config: Dict):
            dict_relevant = dict_config["config_vehicle"]["other"]

            self.id_type_vehicle = dict_relevant["id_type_vehicle"]
            self.id_vehicle = None

            self.length = dict_relevant["length"]
            self.width = dict_relevant["width"]

            self.v_lon_min = dict_relevant["v_lon_min"]
            self.v_lon_max = dict_relevant["v_lon_max"]
            self.v_lat_min = dict_relevant["v_lat_min"]
            self.v_lat_max = dict_relevant["v_lat_max"]

            self.a_lon_max = dict_relevant["a_lon_max"]
            self.a_lon_min = dict_relevant["a_lon_min"]
            self.a_lat_max = dict_relevant["a_lat_max"]
            self.a_lat_min = dict_relevant["a_lat_min"]
            self.a_max = dict_relevant["a_max"]

            self.j_lon_min = dict_relevant["j_lon_min"]
            self.j_lon_max = dict_relevant["j_lon_max"]
            self.j_lat_min = dict_relevant["j_lat_min"]
            self.j_lat_max = dict_relevant["j_lat_max"]

            self.t_react = dict_relevant["t_react"]

            self.radius_disc, self.wheelbase = \
                util_configuration.compute_disc_radius_and_wheelbase(self.length, self.width)

    def __init__(self, dict_config: Dict):
        self.ego = VehicleConfiguration.Ego(dict_config)
        self.other = VehicleConfiguration.Other(dict_config)


class PlanningConfiguration:
    def __init__(self, dict_config: Dict):
        dict_relevant = dict_config["config_planning"]

        self.dt = dict_relevant["dt"]
        self.time_step_start = dict_relevant["time_step_start"]
        self.time_steps_computation = dict_relevant["time_steps_computation"]

        self.p_lon_initial = None
        self.p_lat_initial = None
        self.uncertainty_p_lon = dict_relevant["uncertainty_p_lon"]
        self.uncertainty_p_lat = dict_relevant["uncertainty_p_lat"]

        self.v_lon_initial = None
        self.v_lat_initial = None
        self.v_lon_desired = dict_relevant["v_lon_desired"]
        self.uncertainty_v_lon = dict_relevant["uncertainty_v_lon"]
        self.uncertainty_v_lat = dict_relevant["uncertainty_v_lat"]

        # related to specific planning problem
        self.time_step_initial = None
        self.id_lanelet_initial = None
        self.route = None
        self.reference_path = None
        self.lanelet_network = None

        self.coordinate_system = dict_relevant["coordinate_system"]
        self.reference_point = dict_relevant["reference_point"]
        self.CLCS = None

        self.use_sonia = dict_relevant["use_sonia"]

    def complete_configuration(self, config: Configuration):
        scenario = config.scenario
        planning_problem = config.planning_problem

        self.lanelet_network = scenario.lanelet_network
        self.time_step_initial = planning_problem.initial_state.time_step

        if self.coordinate_system == "CART":
            self.p_lon_initial = planning_problem.initial_state.position[0]
            self.p_lat_initial = planning_problem.initial_state.position[1]
            self.v_lon_initial = planning_problem.initial_state.velocity
            self.v_lat_initial = 0
            self.id_lanelet_initial = 0

        elif self.coordinate_system == "CVLN":
            # plans a route from the initial lanelet to the goal lanelet
            route_planner = RoutePlanner(scenario, planning_problem)
            candidate_holder = route_planner.plan_routes()
            route = candidate_holder.retrieve_first_route()

            self.route = route
            self.reference_path = route.reference_path
            self.id_lanelet_initial = route.list_ids_lanelets[0]

            self.CLCS = util_configuration.create_curvilinear_coordinate_system(self.reference_path)
            p_initial, v_initial = util_configuration.compute_initial_state_CVLN(config)

            self.p_lon_initial, self.p_lat_initial = p_initial
            self.v_lon_initial, self.v_lat_initial = v_initial


class ReachableSetConfiguration:
    def __init__(self, dict_config: Dict):
        dict_relevant = dict_config["config_reachable_set"]

        self.num_threads = dict_relevant["num_threads"]
        self.size_grid = dict_relevant["size_grid"]
        self.size_grid_2nd = dict_relevant["size_grid_2nd"]
        self.radius_terminal_split = dict_relevant["radius_terminal_split"]
        self.prune_nodes_not_reaching_final_time_step = dict_relevant["prune_nodes_not_reaching_final_time_step"]

        self.consider_traffic = dict_relevant["consider_traffic"]
        self.allow_overtaking = dict_relevant["allow_overtaking"]


class DebugConfiguration:
    def __init__(self, dict_config: Dict):
        dict_relevant = dict_config["config_debug"]

        self.save_plots = dict_relevant["save_plots"]
        self.save_config = dict_relevant["save_config"]
        self.verbose_mode = dict_relevant["verbose"]
        self.measure_time = dict_relevant["measure_time"]


class SONIAConfiguration:
    def __init__(self, dict_config: Dict):
        dict_relevant = dict_config["config_spot"]

        self.time_steps_computation_extra = dict_relevant['time_steps_computation_extra']

        self.compute_assumption_m1 = dict_relevant['compute_assumption_m1']
        self.compute_assumption_m2 = dict_relevant['compute_assumption_m2']
        self.compute_assumption_m3 = dict_relevant['compute_assumption_m3']

        self.consider_occlusion = dict_relevant['consider_occlusion']
        self.print_operation_status = dict_relevant['print_operation_status']
        self.print_predicted_velocity_intervals = dict_relevant['print_predicted_velocity_intervals']
        self.update_obstacle_parameters = dict_relevant["update_obstacle_parameters"]
        self.num_threads = dict_relevant['num_threads']
