from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual
from commonroad_reach.data_structure.driving_corridors import DrivingCorridors
import os


def ret_lanelet_ids_list():
    return [3508, 3509, 4473, 4232, 3439, 3440, 4474, 4590, 4240, 4589, 4238, 4234, 4233, 4239, 4591, 4475]


def ret_obstacles_by_id(config, id_list):
    scenario = config.scenario
    obs_list = list()
    for obs_id in id_list:
        obs_list.append(scenario.obstacle_by_id(obs_id))
    return obs_list


def main():
    # ==== build configuration
    # name_scenario = "DEU_Test-1_1_T-1"
    name_scenario = "ARG_Carcarana-1_1_T-1"
    # name_scenario = "ZAM_Tjunction-1_313_T-1"
    # name_scenario = "USA_US101-6_1_T-1"

    config = ConfigurationBuilder.build_configuration(name_scenario, path_root=os.path.join(os.getcwd(), "../../.."))
    config.print_configuration_summary()

    # ==== construct reachability interface and compute reachable sets
    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets()

    reachable_set = reach_interface.reachable_set

    driving_corridors = DrivingCorridors(reachable_set, config)
    longitudinal_driving_corridors = driving_corridors.extract_driving_corridors()
    print("Number of longitudinal driving corridors %s:" % len(longitudinal_driving_corridors))

    # ==== plot computation results
    # plot reachable sets
    # util_visual.plot_scenario_with_reachable_sets(reach_interface, as_svg=False)

    # plot specific driving corridor (dc_idx: idx in list)
    dc_idx = 1
    plot = "3D"

    if plot == "2D":
        util_visual.plot_scenario_with_driving_corridor(longitudinal_driving_corridors[dc_idx], dc_idx, reach_interface,
                                                        time_step_end=reach_interface.time_step_end, animation=True)
    elif plot == "3D":
        # plot 3D corridor

        # util_visual.draw_driving_corridor_3d(longitudinal_driving_corridors[dc_idx], dc_idx, reach_interface,
        #                                      lanelet_ids=None, list_obstacles=None, as_svg=False)

        util_visual.draw_driving_corridor_3d(longitudinal_driving_corridors[dc_idx], dc_idx, reach_interface,
                                             lanelet_ids=ret_lanelet_ids_list(),
                                             list_obstacles=ret_obstacles_by_id(config, [352]),
                                             as_svg=True)

    # plot all driving corridors (complete corridors are plotted in one plot)
    # util_visual.plot_all_driving_corridors(longitudinal_driving_corridors, reach_interface)

if __name__ == "__main__":
    main()
