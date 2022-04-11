from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
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

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 3)
    print("Reachable set computed.")


def test_reachable_set_computation_python_offline():
    # ensure pickled data can be parsed by online reachability
    name_scenario = "DEU_Offline-1_1_T-1"
    dt = 3

    config = ConfigurationBuilder.build_configuration(name_scenario)
    [config.scenario.remove_obstacle(obs) for obs in config.scenario.obstacles]
    config.reachable_set.size_grid = 0.5
    reach_offline = PyGraphReachableSetOffline(config)
    reach_offline.compute(reach_offline.step_start, reach_offline.step_start + dt)
