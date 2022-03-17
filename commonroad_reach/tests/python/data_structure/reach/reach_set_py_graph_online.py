import os
import shutil
import tempfile
import time
import warnings

import matplotlib
import numpy as np
from commonroad.scenario.trajectory import State
from matplotlib import pyplot as plt

try:
    matplotlib.use("TkAgg")
except:
    pass
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.data_structure.reach.reach_set_py_graph_offline import PyGraphReachableSetOffline
from commonroad_reach.data_structure.reach.reach_set_py_graph_online import PyGraphReachableSetOnline
from commonroad_reach.utility import visualization as util_visual


def test_validate_configurations():
    name_scenario = "DEU_Offline-1_1_T-1"

    offline_config = ConfigurationBuilder.build_configuration(name_scenario)
    online_config = ConfigurationBuilder.build_configuration(name_scenario)
    with warnings.catch_warnings(record=True) as w:
        PyGraphReachableSetOnline._validate_configurations(online_config.reachable_set, online_config.vehicle,
                                                           offline_config.reachable_set, offline_config.vehicle)
    assert len(w) == 0

    offline_config.reachable_set.size_grid = offline_config.reachable_set.size_grid * 2
    with warnings.catch_warnings(record=True) as w:
        PyGraphReachableSetOnline._validate_configurations(online_config.reachable_set, online_config.vehicle,
                                                           offline_config.reachable_set, offline_config.vehicle)
    assert len(w) == 1
    assert online_config.reachable_set.size_grid == offline_config.reachable_set.size_grid

    offline_config.vehicle.ego.a_max = offline_config.vehicle.ego.a_max * 2
    with warnings.catch_warnings(record=True) as w:
        PyGraphReachableSetOnline._validate_configurations(online_config.reachable_set, online_config.vehicle,
                                                           offline_config.reachable_set, offline_config.vehicle)
    assert len(w) == 1
    assert online_config.vehicle.ego.a_max == offline_config.vehicle.ego.a_max


def test_offline_reach():
    # ensure pickled data can be parsed by online reachability
    config = name_scenario = "DEU_Offline-1_1_T-1"
    # name_scenario = "ARG_Carcarana-1_1_T-1"
    # name_scenario = "ZAM_Tjunction-1_313_T-1"
    dt = 3

    config = ConfigurationBuilder.build_configuration(name_scenario)
    [config.scenario.remove_obstacle(obs) for obs in config.scenario.obstacles]
    config.reachable_set.size_grid = 0.5
    reach_offline = PyGraphReachableSetOffline(config)

    reach_offline.compute_reachable_sets(reach_offline.time_step_start, reach_offline.time_step_start+dt)


def test_offline_online_compatibility():
    # ensure pickled data can be parsed by online reachability
    offline_dir = tempfile.mktemp(suffix="")
    os.makedirs(offline_dir)
    try:

        config = name_scenario = "DEU_Offline-1_1_T-1"
        dt = 10

        config = ConfigurationBuilder.build_configuration(name_scenario)
        # [config.scenario.remove_obstacle(obs) for obs in config.scenario.dynamic_obstacles]
        # config.reachable_set.size_grid = 0.5
        reach_offline = PyGraphReachableSetOffline(config)

        reach_offline.config.general.path_offline_data = offline_dir
        reach_offline.compute_reachable_sets(reach_offline.time_step_start, reach_offline.time_step_start+dt)

        config.reachable_set.mode = 5
        config.planning_problem.initial_state = State(position=np.array([15,1.5]),
                                                      velocity=15, orientation=0,steering_angle=0.0,yaw_rate=0, slip_angle=0, time_step=0)
        reach_online = PyGraphReachableSetOnline(config)
        reach_online.compute_reachable_sets(1, dt)
    finally:
        shutil.rmtree(offline_dir)
        print("removed")


def online_reach():
    # offline_dir = tempfile.mktemp(suffix="")
    # os.makedirs(offline_dir)
    try:
        # ensure pickled data can be parsed by online reachability
        config = name_scenario = "DEU_Offline-1_1_T-1"
        # name_scenario = "ARG_Carcarana-1_1_T-1"
        # name_scenario = "ZAM_Tjunction-1_313_T-1"
        dt = 18

        config = ConfigurationBuilder.build_configuration(name_scenario)
        config.reachable_set.mode = 5
        config.reachable_set.size_grid = 0.3
        config.planning_problem.initial_state = State(position=np.array([17,1.5]),
                                                      velocity=30, orientation=0,steering_angle=0.0,yaw_rate=0, slip_angle=0, time_step=0)
        config.reachable_set.name_pickle_offline =\
            "/home/klischat/GIT_REPOS/commonroad-reachable-set/output/offline_data/" \
            "offline_33_ms6_dx0.5_amax5.0_vmax30.0_ver0.0.1.pickle"
        reach_interface: ReachableSetInterface = ReachableSetInterface(config)
        reach_online: PyGraphReachableSetOnline = reach_interface._reach
        times = []
        print("start")
        for _ in range(1):
            reach_online.initialize_new_scenario(scenario=config.scenario)
            t0 = time.time()
            reach_online.compute_reachable_sets(0, dt)
            times.append(time.time()-t0)
        print(f"avg time {sum(times)/len(times)}, min {min(times)}, max {max(times)}")
        reach_interface._reachable_set_computed = True
        config.debug.save_plots = True
        util_visual.plot_scenario_with_drivable_area(reach_interface, plot_limits=[0,150,-10,10],time_step_start=1, time_step_end=dt, as_svg=False)
        plt.show(block=True)

    finally:
        # shutil.rmtree(offline_dir)
        print("removed")

if __name__ == "__main__":
    # test_offline_online_compatibility()
    # test_offline_reach()
    online_reach()



