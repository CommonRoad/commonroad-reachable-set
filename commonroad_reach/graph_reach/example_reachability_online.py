from typing import Iterable

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker
from matplotlib import pyplot as plt

from pruningPy.graph_reach.online_graph_reach import *


def plot_scen(scenario=None, reach_set=None, figsize=[16, 10], ttitle='', filename='plot',
              planning_task=None, savefl=False, trajectory=None, scatter=None,
              highlighted_cars: Union[None, List[DynamicObstacle]] = None, obj_list=None, time_interval=(0, 1),
              draw_only=False,
              initial_state: State = None, draw_params: dict = None, pickle_plot: bool = False,
              state: np.ndarray = None):
    # plots the scenario with the specified properties and saves it
    # Plot will only be saved if savefl is set to True

    if draw_only is False:
        # plt.style.use('matplot_ieee.mplstyle')
        # plt.style.use('classic')
        inch_in_cm = 2.54
        fig = plt.figure(figsize=(figsize[0] / inch_in_cm, figsize[1] / inch_in_cm))
        plt.gca().set(title=ttitle)

    rnd = MPRenderer(ax=plt.gca())
    if reach_set != None:
        if type(reach_set) == ReachSetGridOnline:
            reach_set_list = reach_set.create_reachset_list_for_plotting(inverse=False, time_range=time_interval)
            if draw_params is not None and 'reach' in draw_params:
                draw_params_reach = {}
                draw_params_reach['shape'] = draw_params['aarect']
            else:
                draw_params_reach = {'shape': {'facecolor': '#dc2a2a', 'edgecolor': '#dc2a2a', 'zorder': 19}}

            rnd.draw_list(reach_set_list, draw_params=draw_params_reach)
        elif isinstance(reach_set, Iterable):
            # objects = [reach_set[t_] for t_ in range(time_interval[0], time_interval[1] + 1)]
            rnd.draw_list(reach_set, draw_params={'shape': {'facecolor': 'red', 'zorder': 19}})

    if draw_params is None:
        draw_params = {'time_begin': time_interval[0], 'scenario': {'dynamic_obstacle': {'draw_icon': False,
                                                                                         'occupancy': {
                                                                                             'draw_occupancies': 0},
                                                                                         'show_label': True,
                                                                                         'trajectory_steps': 20,
                                                                                         'zorder:': 30}}}
    draw_params['time_begin'] = time_interval[0]
    draw_params['time_end'] = time_interval[1]

    highlight_color = '#ff0000'
    draw_params_highlighted = {'scenario': {'dynamic_obstacle': {'facecolor': highlight_color,
                                                                 'edgecolor': highlight_color,
                                                                 'draw_icon': False,
                                                                 'show_label': False,
                                                                 'trajectory_steps': 20,
                                                                 'zorder:': 30}}}
    if scenario is not None:
        scenario.draw(rnd, draw_params=draw_params)
        # draw_object(scenario, draw_params=draw_params, plot_limits=limits)
    if initial_state is not None:
        initial_state.draw(rnd)

    if highlighted_cars is not None:
        rnd.draw_list(highlighted_cars, draw_params=draw_params_highlighted)

    if planning_task is not None:
        planning_task.draw(rnd)
    if obj_list is not None:
        rnd.draw_list(obj_list, draw_params={'shape': {'facecolor': 'red'}})

    plt.gca().set_aspect('equal')
    if trajectory is not None:
        if isinstance(trajectory, (list, tuple)) == False:
            plt.plot(trajectory[0, :], trajectory[1, :], linewidth=2.0, zorder=np.inf)
        else:
            for traj_i in trajectory:
                plt.plot(traj_i[0, :], traj_i[1, :], linewidth=2.0, zorder=np.inf)

    if scatter is not None:
        # if scatter.shape[]
        if isinstance(scatter, (list, tuple)) == False:
            if len(scatter.shape) == 1:
                scatter = scatter[:, np.newaxis]
            plt.scatter(scatter[0, :], scatter[1, :], linewidth=2.0, zorder=np.inf, color='r')
        else:
            for scatt_i in scatter:
                if len(scatt_i.shape) == 1:
                    scatt_i = scatt_i[:, np.newaxis]
                plt.scatter(scatt_i[0, :], scatt_i[1, :], linewidth=2.0, zorder=np.inf, color='r')

    rnd.render()
    if draw_only is False:
        # pickle matplotlib plot
        if pickle_plot:
            filename_pkl = os.path.splitext(filename)[0] + '.pkl'
            with open(filename_pkl, 'wb') as file:
                pickle.dump(fig, file)

        if savefl:
            plt.savefig(filename)
            plt.close(fig)


folder_reachset = os.getcwd() + '/out/pruning/'
file_offline_reach = '<define path>/28timesteps_dt01.json'
scenario_path = "<define path>"
# filename_scenario = ""
scenario_init, pps = CommonRoadFileReader(scenario_path).open()

collision_checker_params = {'minkowski_sum_circle': False, 'minkowski_sum_circle_radius': 0.5}

parameters_online = {'initial_state': list(pps.planning_problem_dict.values())[0].initial_state,
                     'sensitivity': False,
                     'dynamic_only': False,
                     'collision_checker_params': collision_checker_params,
                     'sparse': True}

cc = create_collision_checker(scenario_init)
# load file once
reachability: ReachSetGridOnline = ReachSetGridOnline.init_from_file(file_offline_reach)
# init scenario
reachability.init_scenario(cc, "test", parameters_online)

# # reset online reachability to avoid re-loading file:
# reachability.reset_online_RA(reset_obstacle_grid=True)
# reachability.init_scenario(other_cc, "test2", parameters_online)
k_max = 28
reachability.forward_steps(k_max)
# refine reachset (optional)
reachability.backward_forward(k_max)

filename = "<define filename>"
plot_scen(scenario=scenario_init,
          reach_set=reachability.create_reachset_list_for_plotting(inverse=False, time_range=[k_max-1, k_max]),
          filename=filename,
          savefl=True,
          time_interval=[k_max-1, k_max])
