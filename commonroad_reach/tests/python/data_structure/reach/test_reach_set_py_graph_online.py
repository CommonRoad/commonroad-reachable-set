import os
import shutil
import tempfile
import time
import warnings
from glob import glob

import matplotlib
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
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
# from commonroad_reach.utility import visualization as util_visual


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
    dt = 5

    config = ConfigurationBuilder.build_configuration(name_scenario)
    config.reachable_set.mode = 5
    config.planning.coordinate_system = "CART"
    [config.scenario.remove_obstacle(obs) for obs in config.scenario.obstacles]
    config.complete_configuration(config.scenario, config.planning_problem)
    config.reachable_set.size_grid = 0.5
    reach_offline = PyGraphReachableSetOffline(config)

    reach_offline.compute(reach_offline.step_start, reach_offline.step_start + dt)


def test_offline_online_compatibility():
    # ensure pickled data can be parsed by online reachability
    offline_dir = tempfile.mktemp(suffix="")
    os.makedirs(offline_dir)
    try:
        name_scenario = "DEU_Offline-1_1_T-1"
        dt = 10

        config = ConfigurationBuilder.build_configuration(name_scenario)
        reach_offline = PyGraphReachableSetOffline(config)

        reach_offline.config.general.path_offline_data = offline_dir
        reach_offline.compute(reach_offline.step_start, reach_offline.step_start + dt)

        config.reachable_set.mode_computation = 4
        config.planning_problem.initial_state = State(position=np.array([15, 1.5]),
                                                      velocity=15, orientation=0, steering_angle=0.0, yaw_rate=0,
                                                      slip_angle=0, time_step=0)
        config.scenario._dynamic_obstacles.clear()
        reach_online = PyGraphReachableSetOnline(config)
        reach_online.compute(1, dt)

    finally:
        shutil.rmtree(offline_dir)


def test_online_reach():
    # ensure pickled data can be parsed by online reachability
    name_scenario = "DEU_Offline-1_1_T-1"
    # name_scenario = "ARG_Carcarana-1_1_T-1"
    # name_scenario = "ZAM_Tjunction-1_313_T-1"
    dt = 33

    config = ConfigurationBuilder.build_configuration(name_scenario)
    config.reachable_set.mode_computation = 3
    config.reachable_set.size_grid = 0.3
    config.planning_problem.initial_state = State(position=np.array([17, 1.5]),
                                                  velocity=30, orientation=0, steering_angle=0.0, yaw_rate=0,
                                                  slip_angle=0, time_step=0)
    reach_interface: ReachableSetInterface = ReachableSetInterface(config)
    reach_online: PyGraphReachableSetOnline = reach_interface._reach
    times = []
    print("start")
    for _ in range(1):
        reach_online.initialize_new_scenario(scenario=config.scenario)
        t0 = time.time()
        reach_online.compute(1, dt)
        times.append(time.time() - t0)
    print(f"avg time {sum(times) / len(times)}, min {min(times)}, max {max(times)}")
    reach_interface._reachable_set_computed = True
    config.debug.save_plots = True


def test_initialize_new_scenario():
    name_scenario = "ARG_Carcarana-1_1_T-1"
    config = ConfigurationBuilder.build_configuration(name_scenario)
    config.reachable_set.mode_computation = 3
    config.reachable_set.n_multi_steps = 5
    config.print_configuration_summary()

    reach_interface = ReachableSetInterface(config)
    times = []
    for path_scenario in glob(os.path.join(config.general.path_scenarios, "*.xml")):
        scenario, pp = CommonRoadFileReader(path_scenario).open()
        reach_interface._reach.initialize_new_scenario(scenario, list(pp.planning_problem_dict.values())[0])
        t=time.time()
        reach_interface.compute_reachable_sets()
        times.append(time.time()-t)

    plt.figure()
    plt.hist(times)
    plt.show()



