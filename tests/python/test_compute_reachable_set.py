from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.data_structure.reach.reach_set_py_graph_offline import PyGraphReachableSetOffline


def test_reachable_set_computation_python_cvln(config: Configuration):
    config.reachable_set.mode_computation = 1
    config.planning.coordinate_system = "CVLN"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_python_cart(config: Configuration):
    config.reachable_set.mode_computation = 1
    config.planning.coordinate_system = "CART"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_cpp_cvln(config: Configuration):
    config.reachable_set.mode_computation = 2
    config.planning.coordinate_system = "CVLN"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_cpp_cart(config: Configuration):
    config.reachable_set.mode_computation = 2
    config.planning.coordinate_system = "CART"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_python_online(config: Configuration):
    config.reachable_set.mode_computation = 3
    config.planning.coordinate_system = "CART"
    import os
    import pathlib
    print(config.general.path_offline_data)
    print("Files in offline_data:")
    for dirpath, dirname, filenames in os.walk(f"{pathlib.Path(__file__).parent.parent.resolve()}"):
        for filename in filenames:
            print(f"{dirpath}: {filename}")

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 3)
    print("Reachable set computed.")


def test_reachable_set_computation_python_offline(config_offline: Configuration):
    # ensure pickled data can be parsed by online reachability
    dt = 3
    [config_offline.scenario.remove_obstacle(obs) for obs in config_offline.scenario.obstacles]
    config_offline.reachable_set.size_grid = 0.5
    reach_offline = PyGraphReachableSetOffline(config_offline)
    reach_offline.compute(reach_offline.step_start, reach_offline.step_start + dt)
