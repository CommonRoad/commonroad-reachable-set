import yaml
import logging
from matplotlib import pyplot as plt

# commonroad_reach
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface

# commonroad_qp_planner
from commonroad_qp_planner.qp_planner import QPPlanner, QPLongDesired, QPLongState, LongitudinalTrajectoryPlanningError, \
    LateralTrajectoryPlanningError
from commonroad_qp_planner.initialization import create_optimization_configuration_vehicle
from commonroad_qp_planner.constraints import LatConstraints, LonConstraints
from commonroad_qp_planner.utility.compute_constraints import longitudinal_position_constraints, \
    lateral_position_constraints
from commonroad_qp_planner.utils import plot_result

# name of scenario
name_scenario = "ZAM_Tutorial-1_2_T-1"

# ****************************************************
# Reachability Analysis
# ****************************************************
# compute reachable sets and driving corridor
config_reach = ConfigurationBuilder.build_configuration(name_scenario)
config_reach.print_configuration_summary()

reach_interface = ReachableSetInterface(config_reach)
reach_interface.compute_reachable_sets()

# ==== plot computation results
# util_visual.plot_scenario_with_reachable_sets(reach_interface)


# ****************************************************
# QP Planner
# ****************************************************
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
lanelets_to_goal = config_reach.planning.route.list_ids_lanelets

# construct qp vehicle configuration
configuration_qp = \
    create_optimization_configuration_vehicle(scenario, planning_problem, settings_qp["vehicle_settings"],
                                              reference_path=reference_path, lanelets_leading_to_goal=lanelets_to_goal)

# Initialize QP Planner
qp_planner = QPPlanner(scenario,
                       planning_problem,
                       config_reach.planning.steps_computation * config_reach.planning.dt,
                       configuration_qp,
                       qp_long_parameters=settings_qp["qp_planner"]["longitudinal_parameters"],
                       qp_lat_parameters=settings_qp["qp_planner"]["lateral_parameters"],
                       verbose=True, logger_level=logging.INFO)


# Longitudinal Trajectory Planning
# Extract lon driving corridor from reachable sets (with/without terminal set constraint)
longitudinal_driving_corridors = reach_interface.extract_driving_corridors(to_goal_region=False)
# Select first longitudinal corridor
lon_dc = longitudinal_driving_corridors[0]

# Derive LonConstraints from driving corridor
s_min, s_max = longitudinal_position_constraints(lon_dc)
c_tv_lon = LonConstraints.construct_constraints(s_min, s_max, s_min, s_max)

# construct longitudinal reference
x_ref = list()
v_des = settings_qp["vehicle_settings"][planning_problem.planning_problem_id]["desired_speed"]
for i in range(len(s_min)):
    x_ref.append(QPLongState(0., v_des, 0., 0., 0.))
desired_lon_states = QPLongDesired(x_ref)

# Plan longitudinal trajectory
traj_lon, status = qp_planner.longitudinal_trajectory_planning(c_long=c_tv_lon, x_des=desired_lon_states)


# Lateral Trajectory Planning
# get longitudinal positions from planned lon trajectory
traj_lon_positions = traj_lon.get_positions()[:, 0]

# Extract lateral driving corridor from lon driving corridor
if status == 'optimal':
    lateral_driving_corridors = reach_interface.extract_driving_corridors(longitudinal_dc=lon_dc,
                                                                          longitudinal_positions=traj_lon_positions)
else:
    raise LongitudinalTrajectoryPlanningError(f'<QPPlanner/_longitudinal_trajectory_planning> '
                                              f'failed, status: {status}')
# select first lateral driving corridor
lat_dc = lateral_driving_corridors[0]

# Derive LatConstraints
d_min, d_max = lateral_position_constraints(lat_dc, lon_dc, traj_lon_positions,
                                            reach_interface._driving_corridor_extractor.steps, configuration_qp)
c_tv_lat = LatConstraints.construct_constraints(d_min, d_max, d_min, d_max)

# Plan lateral trajectory
trajectory_cvln, status = qp_planner.lateral_trajectory_planning(traj_lon, c_tv_lat)
if status == 'optimal':
    pass
else:
    raise LateralTrajectoryPlanningError(f'<QPPlanner/_longitudinal_trajectory_planning> '
                                         f'failed, status: {status}')

# Transform trajectory back to Cartesian
trajectory_cartesian = qp_planner.transform_trajectory_to_cartesian_coordinates(trajectory_cvln)


# ****************************************************
# Visualization and Evaluation
# ****************************************************
# create ego vehicle object
ego_vehicle = trajectory_cartesian.convert_to_cr_ego_vehicle(configuration_qp.width, configuration_qp.length,
                                                             configuration_qp.wheelbase, configuration_qp.vehicle_id)

# plot trajectory
fig, axs = plt.subplots(3)
plot_result(scenario, ego_vehicle, axs[0])

# s_limit
axs[1].plot(list(range(len(s_min))), s_min, color="red")
axs[1].plot(list(range(len(s_max))), s_max, color="red")
axs[1].plot(list(range(len(trajectory_cvln.states) - 1)),
            [state.position[0] for state in trajectory_cvln.states[1:]], color="black")
axs[1].set_xlabel("time step")
axs[1].set_ylabel("s")

# d_limit
axs[2].plot(list(range(len(d_min))), d_min, color="red")
axs[2].plot(list(range(len(d_max))), d_max, color="red")
axs[2].plot(list(range(len(trajectory_cvln.states) - 1)),
            [state.position[1] for state in trajectory_cvln.states[1:]], color="black")
axs[2].set_xlabel("time step")
axs[2].set_ylabel("d")

plt.show()
