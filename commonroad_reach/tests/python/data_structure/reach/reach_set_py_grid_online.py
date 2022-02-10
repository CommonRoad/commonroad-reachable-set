import os
import shutil
import tempfile
import unittest
import matplotlib
import numpy as np
from commonroad.scenario.trajectory import State
from matplotlib import pyplot as plt

matplotlib.use("TkAgg")
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach import reach_interface
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.data_structure.reach.reach_set_py_grid_offline import PyGridOfflineReachableSet
from commonroad_reach.data_structure.reach.reach_set_py_grid_online_discr import PyGridOnlineReachableSet
from commonroad_reach.utility import visualization as util_visual


def offline_online_compatibility():
    # offline_dir = tempfile.mktemp(suffix="")
    # os.makedirs(offline_dir)
    try:
        # ensure pickled data can be parsed by online reachability
        config = name_scenario = "DEU_Offline-1_1_T-1"
        # name_scenario = "ARG_Carcarana-1_1_T-1"
        # name_scenario = "ZAM_Tjunction-1_313_T-1"
        dt = 25

        config = ConfigurationBuilder.build_configuration(name_scenario)
        [config.scenario.remove_obstacle(obs) for obs in config.scenario.dynamic_obstacles]
        config.reachable_set.size_grid = 0.5
        reach_offline = PyGridOfflineReachableSet(config)

        # offline_file = tempfile.mktemp()
        # reach_offline.config.general.path_offline_data = offline_dir
        reach_offline.compute_reachable_sets(reach_offline.time_step_start, reach_offline.time_step_start+dt)
        print([len(r) for t, r in reach_offline.dict_time_to_reachable_set.items()])
        config.planning_problem.initial_state = State(position=np.array([15,1.5]),
                                                      velocity=10, orientation=0,steering_angle=0.0,yaw_rate=0, slip_angle=0, time_step=0)
        reach_online = PyGridOnlineReachableSet(config)
        reach_online.compute_reachable_sets(0, dt)
        reach_interface = ReachableSetInterface(config)
        reach_interface._reach = reach_online
        reach_interface._reachable_set_computed = True
        config.debug.save_plots = False
        util_visual.plot_scenario_with_drivable_area(reach_interface, plot_limits=[0,100,-10,10],time_step_start=dt, time_step_end=dt, as_svg=False)
        plt.show(block=True)
    finally:
        # shutil.rmtree(offline_dir)
        print("removed")

offline_online_compatibility()



