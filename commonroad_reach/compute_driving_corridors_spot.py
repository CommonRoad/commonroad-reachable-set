from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual
import numpy as np

from commonroad.prediction.prediction import Occupancy, SetBasedPrediction
from commonroad.geometry.shape import Polygon, ShapeGroup


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

def main():
    # ==== build configuration
    name_scenario = "DEU_Test-1_1_T-1"
    # name_scenario = "ARG_Carcarana-1_1_T-1"
    # name_scenario = "ZAM_Tjunction-1_313_T-1"
    # name_scenario = "USA_US101-6_1_T-1"

    config = ConfigurationBuilder.build_configuration(name_scenario)
    config.print_configuration_summary()

    # rnd = MPRenderer(figsize=(20, 10))
    # config.scenario.draw(rnd, draw_params={'time_begin': 0})
    # config.planning_problem.draw(rnd)
    # rnd.render()
    # proj_domain_border = np.asarray(config.planning.CLCS.projection_domain())
    # rnd.ax.plot(proj_domain_border[:, 0], proj_domain_border[:, 1], zorder=100, color='orange')
    # plt.show()

    # ==== construct reachability interface and compute reachable sets
    # TODO: move calling of spot in ReachableSetInterface and add flag in config
    spot_prediction(config)
    reach_interface = ReachableSetInterface(config)
    reach_interface.compute_reachable_sets()

    # TODO: extract terminal set from ISS (s, d) box
    longitudinal_driving_corridors = reach_interface.extract_driving_corridors(to_goal_region=True)
    print("Number of longitudinal driving corridors %s:" % len(longitudinal_driving_corridors))

    # plot specific driving corridor (dc_idx: idx in list)
    dc_idx = 0
    plot = "2D"

    if plot == "2D":
        util_visual.plot_scenario_with_driving_corridor(longitudinal_driving_corridors[dc_idx], dc_idx, reach_interface,
                                                        time_step_end=reach_interface.time_step_end, animation=True,
                                                        as_svg=False)
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

    # plot all driving corridors (complete corridors are plotted in one plot)
    # util_visual.plot_all_driving_corridors(longitudinal_driving_corridors, reach_interface)


if __name__ == "__main__":
    main()
