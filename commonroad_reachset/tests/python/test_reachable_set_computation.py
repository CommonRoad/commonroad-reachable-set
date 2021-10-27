from commonroad_reachset.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reachset.data_structure.reach.reachable_set_interface import PyReachableSetInterface
from commonroad_reachset.continuous.continuous_reachability_analysis import \
    ContinuousReachabilityAnalysis


def test_reachable_set_computation(path_config):
    name_scenario = "DEU_Test-1_1_T-1"

    ConfigurationBuilder.set_path_to_config(path_config)
    config = ConfigurationBuilder.build_configuration(name_scenario)

    reachability_analysis = ContinuousReachabilityAnalysis(config)
    reach_manager = PyReachableSetInterface(reachability_analysis)
    reach_manager.compute(1, 5)
    print("Reachable set computed.")
