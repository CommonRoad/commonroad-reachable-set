# TODO: make test case out of this script for PM-model containment

import copy

import matplotlib.pyplot as plt
import numpy as np
from commonroad.scenario.trajectory import State
from commonroad.common.solution import VehicleType
from vehiclemodels.vehicle_parameters import VehicleParameters

from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics

from shapely.geometry.polygon import Polygon

import itertools

from commonroad.common.file_reader import CommonRoadFileReader

import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface


def simulate_state(vehicle_dynamics, current_state, input_state, dt, ego_params):
    state_simulated_next = vehicle_dynamics.simulate_next_state(current_state, input_state, dt, throw=False)
    if (state_simulated_next.velocity ** 2 + state_simulated_next.velocity_y ** 2 > ego_params.v_max ** 2) or \
            state_simulated_next.velocity > ego_params.v_lon_max or state_simulated_next.velocity_y > ego_params.v_lat_max or \
            state_simulated_next.velocity_y < ego_params.v_lat_min or state_simulated_next.velocity < ego_params.v_lon_min:
        return None
    else:
        return state_simulated_next


if __name__ == '__main__':
    # define paths
    scenario_path = "../../../scenarios/DEU_Test-1_1_T-1.xml"

    # read files
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # preprocess scenario
    initial_state = list(planning_problem_set.planning_problem_dict.values())[0].initial_state
    initial_state.orientation = 0.0
    initial_state.acceleration = 0
    vehicle_dynamics = VehicleDynamics.PM(VehicleType.BMW_320i)

    # reachable set
    name_scenario = "DEU_Test-1_1_T-1"

    config = ConfigurationBuilder.build_configuration(name_scenario)
    util_logger.initialize_logger(config)
    config.print_configuration_summary()

    ego_params = config.vehicle.ego

    # ==== construct reachability interface and compute reachable sets
    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets()

    # forward simulation
    list_samples_alon = np.linspace(ego_params.a_lon_min, ego_params.a_lon_max, 11)
    list_samples_alat = np.linspace(ego_params.a_lat_min, ego_params.a_lat_max, 11)

    for t_end in range(1,2):
        fig1, (ax1, ax2, ax3) = plt.subplots(figsize=(15, 5), nrows=1, ncols=3)
        for alon, alat in itertools.product(list_samples_alon, list_samples_alat):
            if alon ** 2 + alat ** 2 > ego_params.a_max ** 2:
                continue
            input_state = State(
                acceleration=alon, acceleration_y=alat, time_step=0
            )
            initial_state_copied = copy.deepcopy(initial_state)
            state_list = []
            s_x_list = []
            s_y_list = []
            v_x_list = []
            v_y_list = []
            for i in range(t_end+1):
                state_list.append(initial_state_copied)
                s_x_list.append(initial_state_copied.position[0])
                s_y_list.append(initial_state_copied.position[1])
                v_x_list.append(initial_state_copied.velocity)
                if hasattr(initial_state_copied, "velocity_y"):
                    v_y_list.append(initial_state_copied.velocity_y)
                else:
                    v_y_list.append(0)
                initial_state_copied = simulate_state(vehicle_dynamics,
                                                      initial_state_copied,
                                                      input_state,
                                                      scenario.dt,
                                                      ego_params)
                if initial_state_copied is None:
                    break

            if len(state_list) == t_end+1:
                ax1.scatter(s_x_list[-1], s_y_list[-1], c="b")
                ax2.scatter(s_y_list[-1], v_y_list[-1], c="g")
                ax3.scatter(s_x_list[-1], v_x_list[-1], c="r")

        # draw drivable area
        # vertices_rs = reach_interface.reachable_set_at_time_step(t_end)[0].position_rectangle().vertices()
        vertices_rs = reach_interface.reachable_set_at_time_step(t_end)[0].position_rectangle.vertices
        polygon = Polygon(vertices_rs)
        x, y = polygon.exterior.xy
        ax1.plot(x, y)
        ax1.grid('on')
        ax1.title.set_text('position rectangle: p_x-p_y')

        # draw lateral polygon
        poly_lat = reach_interface.reachable_set_at_time_step(t_end)[0].polygon_lat
        p_lat, v_lat = poly_lat.exterior.xy
        ax2.plot(p_lat, v_lat)
        ax2.grid('on')
        ax2.title.set_text('lateral polygon: p_y, v_y')

        # draw  longitudinal polygon
        poly_lon = reach_interface.reachable_set_at_time_step(t_end)[0].polygon_lon
        p_lon, v_lon = poly_lon.exterior.xy
        ax3.plot(p_lon, v_lon)
        ax3.grid('on')
        ax3.title.set_text('longitudinal polygon: p_x, v_x')
        plt.show()