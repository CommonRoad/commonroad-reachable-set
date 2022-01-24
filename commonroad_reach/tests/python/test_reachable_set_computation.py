from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import PyReachableSet
from commonroad_reach.continuous.continuous_reachability_analysis import \
    ContinuousReachabilityAnalysis


def test_reachable_set_computation(path_config):
    name_scenario = "DEU_Test-1_1_T-1"

    ConfigurationBuilder.set_path_to_config(path_config)
    config = ConfigurationBuilder.build_configuration(name_scenario)

    reachability_analysis = ContinuousReachabilityAnalysis(config)
    reach_manager = PyReachableSet(reachability_analysis)
    reach_manager.compute_reachable_sets(1, 5)
    print("Reachable set computed.")
