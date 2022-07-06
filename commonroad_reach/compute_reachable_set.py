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
    config.update_configuration()
    util_logger.initialize_logger(config)
    config.print_configuration_summary()

    # ==== compute reachable sets using reachability interface
    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets()

    # # ==== for recomputing reachable sets
    # # config.update_configuration(scenario=,state_initial=,CLCS=)
    # reach_interface.reset(config)
    # # reach_interface.compute_reachable_sets()
    #
    # # ==== extract driving corridors
    # list_corridors_lon = reach_interface.extract_driving_corridors(to_goal_region=False)
    # print(f"\t#DCs (lon): {len(list_corridors_lon)}")
    #
    # # extract lateral corridor from longitudinal corridor
    # corridor_lon = list_corridors_lon[0]
    # list_p_lon = list()
    # for list_nodes_reach in corridor_lon.reach_nodes().values():
    #     p_lon_min = 10000
    #     p_lon_max = -10000
    #     for node_reach in list_nodes_reach:
    #         p_lon_min = min(p_lon_min, node_reach.p_lon_min())
    #         p_lon_max = max(p_lon_max, node_reach.p_lon_max())
    #     p_lon_mid = (p_lon_min + p_lon_max) / 2
    #     list_p_lon.append(p_lon_mid)
    #
    # list_corridors_lat = reach_interface.extract_driving_corridors(to_goal_region=False, corridor_lon=corridor_lon,
    #                                                                list_p_lon=list_p_lon)
    # print(f"\t#DCs (lat): {len(list_corridors_lat)}")

    # ==== plot computation results
    util_visual.plot_scenario_with_reachable_sets(reach_interface)


if __name__ == "__main__":
    main()
