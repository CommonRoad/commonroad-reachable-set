from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface


def test_reachable_set_computation_python_python_cvln(config: Configuration):
    config.reachable_set.mode = 1
    config.planning.coordinate_system = "CVLN"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_python_python_cart(config: Configuration):
    config.reachable_set.mode = 1
    config.planning.coordinate_system = "CART"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_python_cpp_cvln(config: Configuration):
    config.reachable_set.mode = 2
    config.planning.coordinate_system = "CVLN"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_python_cpp_cart(config: Configuration):
    config.reachable_set.mode = 2
    config.planning.coordinate_system = "CART"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_cpp_cpp_cvln(config: Configuration):
    config.reachable_set.mode = 3
    config.planning.coordinate_system = "CVLN"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_cpp_cpp_cart(config: Configuration):
    config.reachable_set.mode = 3
    config.planning.coordinate_system = "CART"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_python_python_online(config: Configuration):
    config.reachable_set.mode = 4
    config.planning.coordinate_system = "CART"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_python_cpp_online(config: Configuration):
    config.reachable_set.mode = 5
    config.planning.coordinate_system = "CART"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")


def test_reachable_set_computation_python_python_offline(config):
    config.reachable_set.mode = 6
    config.planning.coordinate_system = "CART"

    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets(1, 10)
    print("Reachable set computed.")
