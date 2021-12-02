import os
import pickle
import copy
from typing import Union, List
import numpy as np
from typing import Iterable
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.trajectory import State
from commonroad_reach.reach.common.util import project_reachset_and_transform_into_cartesian_coordinate
# from commonroad_reach.reach.reachability_single_vehicle_cpp import ReachSet
from commonroad_reach.reach.reachability_single_vehicle_cpp import ReachabilitySingleVehicle
from commonroad_reach.reach.reachability_single_vehicle_cpp import ReachabilitySingleVehicle as ReachabilitySingleVehicleCPP
from commonroad_reach.visualization.draw_dispatch import draw_object, _create_drawers_dict_pyfvks
from commonroad.visualization.draw_dispatch_cr import _create_drawers_dict
from matplotlib import pyplot as plt
from pylab import *
import scipy.stats

from pruningPy.graph_reach.online_graph_reach import ReachSetGridOnline

def plot_scen(scenario=None, reach_set=None, figsize=[16,10], ttitle='', limits=[-40,80,-45,40], filename='plot',
              planning_task=None, savefl=False, trajectory=None, scatter=None,
              highlighted_cars: Union[None, List[DynamicObstacle]] = None,
              scenario_updater: Union[None, 'ScenarioOptimizer'] = None, obj_list = None, time_interval=[0,-1],draw_only=False,
              initial_state: State=None, draw_params:dict=None, pickle_plot:bool=False, state: np.ndarray=None):
    # plots the scenario with the specified properties and saves it
    # figsize = tuple(height, width), limits = vector[xlim_min x_lim_max y_lim_min y_lim_max]
    # Plot will only be saved if savefl is set to True


    if draw_only is False:
        # plt.style.use('matplot_ieee.mplstyle')
        plt.style.use('classic')
        inch_in_cm = 2.54
        fig = plt.figure(figsize=(figsize[0] / inch_in_cm, figsize[1] / inch_in_cm))
        plt.gca().set(title=ttitle)

    if reach_set != None:
        if type(reach_set) == ReachabilitySingleVehicle:
            for time_idx, drivable_area in reach_set.drivable_area.items():
                # if time_idx >= reach_set.final_time():
                if time_idx >= time_interval[0] and (time_idx < time_interval[1] or time_interval[1]==-1) :
                    draw_object(project_reachset_and_transform_into_cartesian_coordinate(
                        reach_set.reachable_set[time_idx],
                        reach_set.vehicle_configuration.coordinates,
                        reach_set.vehicle_configuration.curvilinear_coordinate_system),
                        draw_params={
                                     'collision': {'polygon': {'facecolor': 'maroon',
                                                               'edgecolor': 'black',
                                                               'zorder': 15,
                                                               'opacity': 0.5}}})
                    # draw_object(drivable_area,
                    #         draw_params={'reach': {'baseset': {'facecolor': 'red', 'zorder': 19}}})
        elif type(reach_set) == ReachabilitySingleVehicleCPP:
            for time_idx, drivable_area in reach_set.drivable_area.items():
                # if time_idx >= reach_set.final_time():
                if time_idx == 0:
                    continue
                if time_idx >= time_interval[0] and (time_idx < time_interval[1] or time_interval[1] == -1):
                    draw_object(project_reachset_and_transform_into_cartesian_coordinate(
                        reach_set.reachable_set[time_idx],
                        reach_set.vehicle_configuration.coordinates,
                        reach_set.vehicle_configuration.curvilinear_coordinate_system),
                        draw_params={
                            'collision': {'polygon': {'facecolor': 'maroon',
                                                      'edgecolor': 'black',
                                                      'zorder': 15,
                                                      'opacity': 0.5}}})
        elif type(reach_set) == ReachSetGridOnline:
            reach_set_list = reach_set.create_reachset_list_for_plotting(inverse=False, time_range=time_interval)
            if draw_params is not None and 'reach' in draw_params:
                draw_params_reach = {}
                draw_params_reach['shape'] = draw_params['aarect']
            else:
                draw_params_reach = {'shape': {'facecolor': '#dc2a2a','edgecolor':'#dc2a2a', 'zorder': 19}}

            draw_object(reach_set_list, draw_params=draw_params_reach)
        elif isinstance(reach_set, Iterable):
            objects = [reach_set[t_] for t_ in range(time_interval[0], time_interval[1] + 1)]
            draw_object(objects, draw_params={'shape': {'facecolor': 'red', 'zorder': 19}})

    if draw_params is None:
        draw_params = {'time_begin':time_interval[0], 'scenario': {'dynamic_obstacle': {'draw_icon': False,
                                                        'occupancy':{'draw_occupancies': 0},
                                                        'show_label': True,
                                                        'trajectory_steps': 20,
                                                        'zorder:': 30}}}
    draw_params['time_begin'] = time_interval[0]
    draw_params['time_end'] = time_interval[1]

    highlight_color = '#ff0000'
    draw_params_highlighted = {'scenario':{'dynamic_obstacle': {'facecolor': highlight_color,
                                                   'edgecolor': highlight_color,
                                                   'draw_icon': False,
                                                   'show_label': False,
                                                   'trajectory_steps': 20,
                                                   'zorder:': 30}}}
    if scenario is not None:
        draw_object(scenario, draw_params=draw_params)
        plt.gca().autoscale()
        # draw_object(scenario, draw_params=draw_params, plot_limits=limits)
    if initial_state is not None:
        draw_object(initial_state)

    if highlighted_cars is not None:
        draw_object(highlighted_cars, draw_params=draw_params_highlighted)

    if planning_task is not None:
        draw_object(planning_task)
    if obj_list is not None:
        draw_object(obj_list, draw_params={'shape': {'facecolor': 'red'}})

    plt.gca().set_aspect('equal')
    if trajectory is not None:
        if isinstance(trajectory, (list, tuple)) == False:
            plt.plot(trajectory[0,:],trajectory[1,:],linewidth=2.0,zorder=np.inf)
        else:
            for traj_i in trajectory:
                plt.plot(traj_i[0, :], traj_i[1, :],linewidth=2.0,zorder=np.inf)

    if scatter is not None:
        # if scatter.shape[]
        if isinstance(scatter, (list, tuple)) == False:
            if len(scatter.shape) == 1:
                scatter = scatter[:,np.newaxis]
            plt.scatter(scatter[0,:],scatter[1,:],linewidth=2.0,zorder=np.inf,color='r')
        else:
            for scatt_i in scatter:
                if len(scatt_i.shape) == 1:
                    scatt_i = scatt_i[:, np.newaxis]
                plt.scatter(scatt_i[0, :], scatt_i[1, :],linewidth=2.0,zorder=np.inf,color='r')

    if draw_only is False:
        # pickle matplotlib plot
        if pickle_plot:
            filename_pkl = os.path.splitext(filename)[0] + '.pkl'
            with open(filename_pkl,'wb') as file:
                pickle.dump(fig,file)

        if savefl:
            plt.savefig(filename)
            plt.close(fig)


def plot_scen_multiRA(scenario=None, reach_set=None, figsize=[16,10], ttitle='', limits=None, filename='plot', plot_lim=3,
                      planning_task=None, savefl=False, test=False, trajectory=None, scatter=None,
                      highlighted_cars: Union[None, List[DynamicObstacle]] = None,
                      boundary_conditions: Union[None] = None, obj_list = None, obj_list1=None, obj_list2=None, t0=0,
                      file_extensions=['svg']):
    if not test:
        # set_non_blocking()
        # plt.style.use('matplot_ieee.mplstyle')
        # plt.style.use('classic')
        inch_in_cm = 2.54
        plt.figure(figsize=(figsize[0] / inch_in_cm, figsize[1] / inch_in_cm))
        # draw_object(scenario.lanelet_network)
        draw_params = {'time_begin': t0-1,'time_end': t0-1,'scenario': {'dynamic_obstacle': {'draw_icon': False,
                                                         'facecolor': '#727272',
                                                         'edgecolor': 'black',
                                                        'occupancy':{'draw_occupancies': 0},
                                                        'show_label': False,
                                                        'trajectory_steps': 0,
                                                        'zorder:': 30}}}

        highlight_color = '#ff0000'
        draw_params_highlighted = {'scenario':{'dynamic_obstacle': {'facecolor': highlight_color,
                                                       'edgecolor': highlight_color,
                                                       'draw_icon': False,
                                                       'show_label': False,
                                                       'trajectory_steps': 0,
                                                       'zorder:': 30}}}
        if scenario is not None:
            draw_object(scenario, draw_params=draw_params)
        if highlighted_cars is not None:
            draw_object(highlighted_cars, draw_params=draw_params_highlighted)

        if planning_task is not None:
            draw_object(planning_task)
        if obj_list is not None:
            draw_object(obj_list, draw_params={'shape': {'facecolor': '#520a0a','linewidth':0}})
        if obj_list1 is not None:
            draw_object(obj_list1, draw_params={'shape': {'facecolor': '#981e1e','linewidth':0}})
        if obj_list2 is not None:
            draw_object(obj_list2, draw_params={'shape': {'facecolor': '#dc2a2a','linewidth':0}})

        if reach_set != None:
            # for time_idx in range(reach_set.initial_time(), reach_set.final_time() + 1):
            #     if time_idx >= reach_set.final_time():
            draw_object(reach_set,
                    draw_params={'reach': {'baseset': {'facecolor': '#1d7eea','opacity':0.55}}})
        # for i in range(0, len(cars)):
        #     draw_object(cars[i])
        plt.gca().set_aspect('equal')
        plt.gca().set(title=ttitle)
        plt.xticks([], [])
        plt.yticks([], [])
        # if limits is not None:
        plt.xlim(limits[0], limits[1])
        plt.ylim(limits[2], limits[3])

        plt.tight_layout()
        if trajectory is not None:
            if isinstance(trajectory, (list, tuple)) == False:
                plt.plot(trajectory[0,:],trajectory[1,:],linewidth=2.0,zorder=np.inf)
            else:
                for traj_i in trajectory:
                    plt.plot(traj_i[0, :], traj_i[1, :],linewidth=2.0,zorder=np.inf)

        if scatter is not None:
            # if scatter.shape[]
            if isinstance(scatter, (list, tuple)) == False:
                if len(scatter.shape) == 1:
                    scatter = scatter[:,np.newaxis]
                plt.scatter(scatter[0,:],scatter[1,:],linewidth=2.0,zorder=np.inf,color='r')
            else:
                for scatt_i in scatter:
                    if len(scatt_i.shape) == 1:
                        scatt_i = scatt_i[:, np.newaxis]
                    plt.scatter(scatt_i[0, :], scatt_i[1, :],linewidth=2.0,zorder=np.inf,color='r')

        if savefl:
            # try:
            for ext in file_extensions:
                plt.savefig(filename + '.' + ext)
            # except:
            #     print('saving failed')
        else:
            plt.show()

    else:
        plt.figure(figsize=(5, 5))
        plt.xlim([limits[0], limits[1]])
        plt.ylim([limits[2], limits[3]])
        if reach_set != None:
            for time_idx in range(reach_set.initial_time(), plot_lim + 1):
                draw_object(reach_set.at_time(time_idx),
                            draw_params={'no_parent': {'reach': {'aarect': {'facecolor': "ff3333"}}}})
        # draw_object(scenario.lanelet_network)
        if scenario is not None:
            cars = scenario.get_active_traffic()
        draw_object(scenario)
        for i in range(len(cars)):
            draw_object(cars[i])


def plot_graph(x, y: Union[List,np.ndarray], y_max=100, filename='unnamed_plot.eps', savefl=False, labels=None, ttitle='', ylabel='drivable area', xlabel='time', plotting=True, stack_axis='x', fig=None, y_limits=None):
    """

    :param x:
    :param y:
    :param y_max:
    :param filename:
    :param savefl:
    :param labels:
    :param ttitle:
    :param ylabel:
    :param xlabel:
    :param plotting:
    :param stack_axis: axis for stacking multiple y data arrays (x or y)
    :return:
    """
    # plt.style.use('matplot_ieee.mplstyle')
    if fig is None:
        plt.style.use('classic')
        inch_in_cm = 2.54
        fig=plt.figure(figsize=(12 / inch_in_cm, 6.3 / inch_in_cm))
        plt.rc('axes', labelsize=12)
        plt.gca().set(xlabel=xlabel, ylabel=ylabel)
        plt.tight_layout()
        plt.gca().grid(False)

    x_0 = 0
    if isinstance(y, (list, tuple))==False:
        if x is None or x is []:
            x = range(0, len(y))
        plt.plot(x, y,ms=3.0,marker='o')
    else:
        ii=0
        for i_y in y:
            if labels is not None:
                if ii < len(labels):
                    label = labels[ii]
                else:
                    label = labels[-1]
            else:
                label=None

            if x is None or x is []:
                nx=np.size(i_y)
                _x = np.array(list(range(0, np.size(i_y))))
            else:
                _x = x

            if stack_axis == 'x':
                _x = _x + x_0

            try:
                plt.plot(_x, i_y, label=label)
            except:
                print('GRAPH FAILED! ! ! ! ! ! !! ! ')
            ii+=1
            if stack_axis == 'x':
                x_0 = _x[-1]
            plt.legend(loc='upper left')

    plt.gca().set_yscale('log')
    if isinstance(y, list):
        y = np.array(y)
    # plt.ylim([np.min(y[np.isfinite(y)]), np.max(y[np.isfinite(y)])])
    # plt.gca().set(title=ttitle)
    if y_limits is not None:
        plt.ylim(y_limits)
    plt.tight_layout()
    if savefl:
        plt.savefig(filename)
    else:
        plt.show()

    return fig


def plot_box_graph(x, y: Union[List,np.ndarray], y_max=100, filename='unnamed_plot.eps', savefl=False, labels=None, ttitle='', ylabel='drivable area', xlabel='time', plotting=True, stack_axis='x', fig=None):
    """

    :param x:
    :param y:
    :param y_max:
    :param filename:
    :param savefl:
    :param labels:
    :param ttitle:
    :param ylabel:
    :param xlabel:
    :param plotting:
    :param stack_axis: axis for stacking multiple y data arrays (x or y)
    :return:
    """
    # plt.style.use('matplot_ieee.mplstyle')
    if fig is None:
        plt.style.use('classic')
        inch_in_cm = 2.54
        fig=plt.figure(figsize=(12 / inch_in_cm, 6.3 / inch_in_cm))
        plt.rc('axes', labelsize=12)
        plt.gca().set(xlabel=xlabel, ylabel=ylabel)
        plt.tight_layout()
        plt.gca().grid(False)

    x_0 = 0
    bp = plt.boxplot(y, notch=0, sym='b+', vert=1, whis=1.5,
                     positions=None, widths=0.6)

    plt.gca().set(title=ttitle)
    plt.tight_layout()
    if savefl:
        plt.savefig(filename)
    else:
        plt.show()

    return fig

