import yaml
import logging

# commonroad_reach
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual

# commonroad_qp_planner
from commonroad_qp_planner.qp_planner import QPPlanner, QPLongDesired, QPLongState
from commonroad_qp_planner.initialization import create_optimization_configuration_vehicle
from commonroad_qp_planner.constraints import LatConstraints, LonConstraints, TVConstraints


# name of scenario
name_scenario = "ZAM_Tutorial-1_2_T-1"

# **************************
# Reachability Analysis
# **************************
# compute reachable sets and driving corridor
config_reach = ConfigurationBuilder.build_configuration(name_scenario)
config_reach.print_configuration_summary()

reach_interface = ReachableSetInterface(config_reach)
reach_interface.compute_reachable_sets()

# ==== plot computation results
util_visual.plot_scenario_with_reachable_sets(reach_interface)


# **************************
# QP Planner
# **************************
# load qp YAML settings
yaml_file = "/home/gerald/Documents/CommonRoad/cps/commonroad-qp-planner/test/config_files/config_%s.yaml" \
            % name_scenario
with open(yaml_file, 'r') as stream:
    try:
        settings_qp = yaml.load(stream, Loader=yaml.Loader)
    except yaml.YAMLError as exc:
        print(exc)

# get scenario, planning problem, reference path from reach config
scenario = config_reach.scenario
planning_problem = config_reach.planning_problem
reference_path = config_reach.planning.reference_path
lanelets_to_goal = config_reach.planning.id_lanelet_initial

# construct qp vehicle configuration
configuration_qp = \
    create_optimization_configuration_vehicle(scenario, planning_problem, settings_qp, reference_path=reference_path,
                                              lanelets_leading_to_goal=lanelets_to_goal)

# initialize QP Planner
qp_planner = QPPlanner(scenario,
                       planning_problem,
                       settings_qp["general_planning_settings"]["time_horizon"],
                       configuration_qp,
                       qp_long_parameters=settings_qp["qp_planner"]["longitudinal_parameters"],
                       qp_lat_parameters=settings_qp["qp_planner"]["lateral_parameters"],
                       verbose=True, logger_level=logging.INFO)

# Steps
# Extract lon driving corridor from reachable sets (with/without terminal set constraint)
longitudinal_driving_corridors = reach_interface.extract_driving_corridors(to_goal_region=True)

# Loop over longitudinal driving corridors

# Derive LonConstraints from driving corridor
# TODO see longitudinal_position_constraints()

# set longitudinal TV constraints
c_tv_lon = LonConstraints.construct_constraints(s_min, s_max, s_min, s_max)

# construct longitudinal reference
v_des = settings_qp["vehicle_settings"][self.planning_problem.planning_problem_id]["desired_speed"]
x_ref = list()
for i in range(len(s_min)):
    # TODO why not s position in x_ref??
    x_ref.append(QPLongState(0., v_des, 0., 0., 0.))
desired_lon_states = QPLongDesired(x_ref)

# longitudinal trajectory planning
traj_lon, status = qp_planner.longitudinal_trajectory_planning(c_long=c_tv_lon, x_des=desired_lon_states)

# get curvilinear positions from planned lon trajectory
# TODO

# Extract lateral driving corridor from lon driving corridor
# TODO
lateral_driving_corridors = reach_interface.extract_driving_corridors(longitudinal_dc=xxx, longitudinal_positions=xxx)

# Derive LatConstraints from lateral driving corridor
# TODO see lateral_position_constraints()

# set lateral TV constraints
# TODO
c_tv_lat = LatConstraints.construct_constraints(d_min, d_max, d_min, d_max)

# lateral trajectory planning
trajectory, status = qp_planner.lateral_trajectory_planning(traj_lon, c_tv_lat)

# Transform trajectory back to Cartesian
trajectory_cartesian = qp_planner.transform_trajectory_to_cartesian_coordinates(trajectory)
ego_vehicle = trajectory_cartesian.convert_to_cr_ego_vehicle(configuration_qp.width, configuration_qp.length,
                                                             configuration_qp.wheelbase, configuration_qp.vehicle_id)


# **************************
# Visualization
# **************************
