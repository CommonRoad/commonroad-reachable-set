from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface_online import OnlineReachableSetInterface
from commonroad_reach.utility import offline_generation as util_offline
from commonroad_reach.utility import visualization as util_visual


def main():
    # ==== build configuration
    # name_scenario = "DEU_Test-1_1_T-1"
    name_scenario = "ARG_Carcarana-1_1_T-1"
    # name_scenario = "ZAM_Tjunction-1_313_T-1"
    config = ConfigurationBuilder.build_configuration(name_scenario)

    reach_interface = OnlineReachableSetInterface(config, "offline_15_0.5_5.0_30.0.pickle")
    reach_interface.compute_reachable_sets()

    util_visual.plot_scenario_with_reachable_sets(reach_interface, as_svg=False)
    # util_offline.save_offline_computation(config, reach_interface)


if __name__ == "__main__":
    main()
