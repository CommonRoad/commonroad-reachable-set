import commonroad_reach.utility.logger as util_logger
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual


def main():
    # ==== specify scenario
    name_scenario = "DEU_Test-1_1_T-1"
    # name_scenario = "ARG_Carcarana-1_1_T-1"
    # name_scenario = "USA_US101-6_1_T-1"
    # name_scenario = "ZAM_Intersection-1_1_T-1"

    # ==== build configuration
    config = ConfigurationBuilder.build_configuration(name_scenario)
    util_logger.initialize_logger(config)
    config.print_configuration_summary()
    # ==== compute reachable sets using reachability interface
    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets()

    # ==== plot computation results
    util_visual.plot_scenario_with_reachable_sets(reach_interface)


if __name__ == "__main__":
    main()
