from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
import commonroad_reach.utility.logger as util_logger
from commonroad_reach.utility import visualization as util_visual


def main():
    # ==== specify scenario
    # name_scenario = "DEU_Test-1_1_T-1"
    name_scenario = "ARG_Carcarana-1_1_T-1"
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

    # ==== extract driving corridors
    longitudinal_driving_corridors = reach_interface.extract_driving_corridors(to_goal_region=False)

    print("Number of longitudinal driving corridors %s:" % len(longitudinal_driving_corridors))

    # plot specific driving corridor (dc_idx: idx in list)
    dc_idx = 0
    plot = "3D"

    if plot == "2D":
        util_visual.plot_scenario_with_driving_corridor(longitudinal_driving_corridors[dc_idx], dc_idx, reach_interface,
                                                        save_gif=True, as_svg=False)
        util_visual.draw_driving_corridor_2d(longitudinal_driving_corridors[dc_idx], dc_idx, reach_interface)
    elif plot == "3D":
        # plot 3D corridor
        # list_obstacle_ids = None
        list_obstacle_ids = [352]
        util_visual.draw_driving_corridor_3d(longitudinal_driving_corridors[dc_idx], dc_idx, reach_interface,
                                             list_obstacle_ids=list_obstacle_ids)


if __name__ == "__main__":
    main()
