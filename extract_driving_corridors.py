from typing import Optional
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
import commonroad_reach.utility.logger as util_logger
from commonroad_reach.utility import visualization as util_visual


def main():
    # ==== specify scenario
    name_scenario = "DEU_Test-1_1_T-1"
    # name_scenario = "ARG_Carcarana-1_1_T-1"
    # name_scenario = "USA_US101-6_1_T-1"
    # name_scenario = "ZAM_Intersection-1_1_T-1"

    # ==== set root path: set to absolute path of commonroad-reachable-set
    path_root: Optional[str] = None

    # ==== build configuration
    config = ConfigurationBuilder(path_root=path_root).build_configuration(name_scenario)
    config.update()
    util_logger.initialize_logger(config)
    config.print_configuration_summary()

    # ==== compute reachable sets using reachability interface
    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets()

    # ==== extract driving corridors
    longitudinal_driving_corridors = reach_interface.extract_driving_corridors(to_goal_region=False)

    # plot specific driving corridor (dc_idx: idx in list)
    dc_idx = 0
    plot = "2D"

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

    # ==== extract driving corridors
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


if __name__ == "__main__":
    main()
