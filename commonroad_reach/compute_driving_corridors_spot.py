from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual
import numpy as np

from commonroad.prediction.prediction import Occupancy, SetBasedPrediction
from commonroad.geometry.shape import Rectangle, Polygon, ShapeGroup


def create_shape_group(polygon_vertices):
    shape_group = ShapeGroup([])
    j = 1  # iterator over polygon_vertices
    b = 0  # index to select polygon_vertices that are the start of a new polygon
    while j < len(polygon_vertices):
        compare_vertice = polygon_vertices[b]  # first vertice of next polygon
        if compare_vertice[0] == polygon_vertices[j][0] and \
                compare_vertice[1] == polygon_vertices[j][1]:
            if (j + 1) - b < 3:  # polygon has less than 3 vertices
                b += 1  # try next vertex as first vertex (in case of equal vertices directly
                # after each other)
            else:
                # polygon has at least 3 vertices
                shape_obj = Polygon(np.array(polygon_vertices[b:j + 1]))
                # update list
                shape_group.shapes.append(shape_obj)
                j += 1
                b = j
        j += 1
    assert b == j - 1

    return shape_group


def spot_result_to_SetBasedPrediction(spot_results, scenario, time_step_start: int,
                                      time_step_end: int):
    """
    Function to convert the result of spot.doOccupancyPrediction to CommonRoad SetBasedPrediction class

    :param spot_results: List[List[obstacle ID, ([lane_idx][1,2] := velocities (min, max), vertices)]]
    :param scenario: Scenario for which the TrajectoryPrediction of dynamic obstacles should be replaced with SetBasedPrediction
    :param time_step_start: int, initial time step for the prediction
    :param time_step_end: int, final time step for the prediction
    :return:
    """
    for obstacle_id, result_all_ts in spot_results:
        # list of predicted occupancies of the obstacle
        list_occupancies = []
        for t, result_ts in zip(range(time_step_start, time_step_end), result_all_ts):
            polygon_vertices = result_ts[1]
            shape_group = create_shape_group(polygon_vertices)
            list_occupancies.append(Occupancy(t, shape_group))

        obstalce = scenario.obstacle_by_id(obstacle_id)
        obstalce.prediction = SetBasedPrediction(time_step_start, list_occupancies)


def spot_prediction(config):
    import spot
    spot_scenario_id = 1
    spot_update_dict = {'Vehicle': {
        0: {
            'a_max': 4.0,
            'compute_occ_m1': True,
            'compute_occ_m2': True,
            'compute_occ_m3': False,
            'onlyInLane': False
        }}}
    # default FOV (no occlusions considered)
    field_of_view = np.empty([0, 2], float)
    # register spot scenario
    spot.registerScenario(spot_scenario_id, config.scenario.lanelet_network, config.scenario.dynamic_obstacles,
                          [config.planning_problem], field_of_view)
    # update obstacle parameters
    spot.updateProperties(spot_scenario_id, spot_update_dict)

    # compute absolute start and end time
    time_step_start = config.planning.time_step_start
    time_step_end = config.planning.time_step_start + config.planning.time_steps_computation
    start_time = time_step_start * config.planning.dt
    end_time = time_step_end * config.planning.dt

    # compute set-based prediction
    spot_results = spot.doOccupancyPrediction(spot_scenario_id, float(start_time), float(config.planning.dt), float(end_time), 4)
    spot_result_to_SetBasedPrediction(spot_results, config.scenario, time_step_start, time_step_end)
    spot.removeScenario(spot_scenario_id)

def get_bounding_box_from_iss(iss_results_list):
    # TODO: extract s-d 2d shape for ego lane
    # s_min, s_max = np.inf, -np.inf
    ego_lane_result_dict = iss_results_list[-1].iss_lanelet_result_dict[iss_results_list[-1].ego_lanelet_id].slice_res
    d_samples = list(ego_lane_result_dict.keys())
    d_min = np.min(d_samples)
    d_max = np.max(d_samples)
    ego_lane_result = list(ego_lane_result_dict.values())[0]
    polygon_vertices_s = ego_lane_result.braking_result[-1].poly_vertices[:, 0]
    s_min = np.min(polygon_vertices_s)
    s_max = np.max(polygon_vertices_s)
    # for d_result in ego_lane_result:
    #     polygon_vertices_s = d_result.braking_result[0].poly_vertices[0, :]
    #     s_min = np.min([s_min, np.min(polygon_vertices_s)])
    #     s_max = np.max([s_max, np.max(polygon_vertices_s)])

    return Rectangle(length=s_max-s_min, width=d_max-d_min, center=np.array([0.5*(s_min+s_max), 0.5*(d_min+d_max)])), \
           [s_min, s_max, d_min, d_max]

def main():
    # ==== build configuration
    name_scenario = "DEU_Test-1_1_T-1"
    # name_scenario = "ARG_Carcarana-1_1_T-1"
    # name_scenario = "ZAM_Tjunction-1_313_T-1"
    # name_scenario = "USA_US101-6_1_T-1"

    # config = ConfigurationBuilder.build_configuration(name_scenario)
    # config.print_configuration_summary()

    from commonroad.common.file_reader import CommonRoadFileReader
    scenario, planning_problem_set = CommonRoadFileReader("../scenarios/DEU_Test-1_1_T-1.xml").open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    time_step_start = 0
    time_steps_computation = 40

    # rnd = MPRenderer(figsize=(20, 10))
    # config.scenario.draw(rnd, draw_params={'time_begin': 0})
    # config.planning_problem.draw(rnd)
    # rnd.render()
    # proj_domain_border = np.asarray(config.planning.CLCS.projection_domain())
    # rnd.ax.plot(proj_domain_border[:, 0], proj_domain_border[:, 1], zorder=100, color='orange')
    # plt.show()

    # TODO: extract terminal set from ISS (s, d) box
    # compute ISS
    from commonroad_iss.iss_highD.iss_highD import ISSHighD
    from commonroad_iss.data_structure.commonroad_iss_configurations import CommonRoadISSConfigurations
    # TODO: check difference between default configs in iss and safe wrapper repo
    default_config_path = "/home/xiao/projects/safe_RL/commonroad-reachable-set/" \
                          "external/commonroad-invariably-safe-set/configurations/defaults"
    iss_config = CommonRoadISSConfigurations.load_default_configuration(default_config_path)
    iss_highD = ISSHighD(scenario, planning_problem, iss_config)
    iss_results_list, computation_time_ms, _ = iss_highD.compute(
        ego_state=planning_problem.initial_state,
        scenario_time_step=0,
        iss_horizon_start=time_step_start + time_steps_computation,
        iss_horizon_length=1,
        bounding_box=(-500., 500., -50., 50.),
    )
    iss_rectangle, bb_box = get_bounding_box_from_iss(iss_results_list)

    # TODO: create config from scratch
    ego_lanelet_id = iss_results_list[-1].ego_lanelet_id
    ego_CLCS = iss_results_list[-1].iss_lanelet_result_dict[ego_lanelet_id].cvln_lanelet.CLCS
    config = ConfigurationBuilder.construct_configuration_for_given_scenario(
        scenario=scenario, planning_problem=planning_problem, CLCS=ego_CLCS
    )

    # # set config.planning.CLCS
    # original_ref_path = np.array(config.planning.CLCS.reference_path())
    # config.planning.CLCS = ego_CLCS
    # new_ref_path = np.array(config.planning.CLCS.reference_path())
    #
    # from commonroad_reach.utility import configuration as util_configuration
    # p_initial, v_initial = util_configuration.compute_initial_state_cvln(config)
    #
    # config.planning.p_lon_initial, config.planning.p_lat_initial = p_initial
    # config.planning.v_lon_initial, config.planning.v_lat_initial = v_initial
    # config.planning.reference_path = new_ref_path

    # import matplotlib.pyplot as plt
    # # plt.plot(original_ref_path[:, 0], original_ref_path[:, 1], marker="+", color="black")
    # plt.plot(new_ref_path[:, 0], new_ref_path[:, 1], marker="*", color="blue")
    # plt.show()

    # ==== construct reachability interface and compute reachable sets
    # TODO: move calling of spot in ReachableSetInterface and add flag in config
    spot_prediction(config)
    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets()

    # util_visual.plot_scenario_with_reachable_sets(reach_interface, as_svg=False)
    # longitudinal_driving_corridors = reach_interface.extract_driving_corridors(to_goal_region=True)
    longitudinal_driving_corridors = reach_interface.extract_driving_corridors(terminal_set=iss_rectangle)
    print("Number of longitudinal driving corridors %s:" % len(longitudinal_driving_corridors))

    # plot specific driving corridor (dc_idx: idx in list)
    dc_idx = 0
    plot = "2D"

    if plot == "2D":
        if len(longitudinal_driving_corridors) > 0:
            util_visual.plot_scenario_with_driving_corridor(longitudinal_driving_corridors[dc_idx], dc_idx, reach_interface,
                                                            time_step_end=reach_interface.time_step_end, animation=True,
                                                            as_svg=False, terminal_set=bb_box)
        else:
            # plot scenario to debug
            import matplotlib.pyplot as plt
            from commonroad.visualization.mp_renderer import MPRenderer

            plot_limits = config.debug.plot_limits
            renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))
            config.scenario.draw(renderer, draw_params={"dynamic_obstacle": {"draw_icon": True}, "time_begin": 0})

            # draw terminal_set
            from commonroad.geometry.shape import Polygon
            # convert terminal_set to Cartesian
            ccosy = config.planning.CLCS
            # ts_x, ts_y = terminal_set.shapely_object.exterior.coords.xy
            # terminal_set_vertices = [vertex for vertex in zip(ts_x, ts_y)]
            transformed_rectangle, triangle_mesh = ccosy.convert_rectangle_to_cartesian_coords(
                bb_box[0], bb_box[1], bb_box[2], bb_box[3])  #
            # create CommonRoad Polygon
            terminal_shape = Polygon(vertices=np.array(transformed_rectangle))
            # terminal_shape.draw(renderer,
            #                     draw_params={"polygon":
            #                         {
            #                             "opacity": 1.0,
            #                             "linewidth": 0.5,
            #                             "facecolor": "#f1b514",
            #                             "edgecolor": "#302404",
            #                             "zorder": 15
            #                         }})
            # config.planning_problem.initial_state.draw(renderer)
            config.planning_problem.draw(renderer)
            # plot
            plt.rc("axes", axisbelow=True)
            ax = plt.gca()
            ax.plot(original_ref_path[:, 0], original_ref_path[:, 1], marker="+", color="black", zorder=50)
            ax.plot(new_ref_path[:, 0], new_ref_path[:, 1], marker="+", color="blue", zorder=50)
            ax.set_aspect("equal")

            plt.margins(0, 0)
            renderer.render(show=True)

    elif plot == "3D":
        # plot 3D corridor

        # util_visual.draw_driving_corridor_3d(longitudinal_driving_corridors[dc_idx], dc_idx, reach_interface,
        #                                      lanelet_ids=None, list_obstacles=None, as_svg=False)

        util_visual.draw_driving_corridor_3d(longitudinal_driving_corridors[dc_idx], dc_idx, reach_interface,
                                             lanelet_ids=None, list_obstacles=None, as_svg=False)

        # util_visual.draw_driving_corridor_3d(longitudinal_driving_corridors[dc_idx], dc_idx, reach_interface,
        #                                      lanelet_ids=ret_lanelet_ids_list(),
        #                                      list_obstacles=ret_obstacles_by_id(config, [352]),
        #                                      as_svg=True)

    # # plot all driving corridors (complete corridors are plotted in one plot)
    # util_visual.plot_all_driving_corridors(longitudinal_driving_corridors, reach_interface)


if __name__ == "__main__":
    main()
