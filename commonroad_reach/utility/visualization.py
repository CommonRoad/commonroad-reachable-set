import logging
import os
from copy import deepcopy
from typing import Tuple, Union, List, Dict

logger = logging.getLogger(__name__)
logging.getLogger('PIL').setLevel(logging.WARNING)
logging.getLogger('matplotlib.font_manager').setLevel(logging.WARNING)

from pathlib import Path
import imageio
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, PolyCollection

from commonroad.geometry.shape import Polygon, Rectangle
from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import coordinate_system as util_coordinate_system
from commonroad_reach.utility.general import create_lanelet_network_from_ids
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon


def plot_scenario_with_reachable_sets(reach_interface: ReachableSetInterface, figsize: Tuple = None,
                                      step_start: int = 0, step_end: int = 0, steps: List[int] = None,
                                      plot_limits: Union[List] = None, path_output: str = None,
                                      save_gif: bool = True, duration: float = None):
    config = reach_interface.config
    scenario = config.scenario
    planning_problem = config.planning_problem
    ref_path = config.planning.reference_path

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    figsize = figsize if figsize else (25, 15)
    plot_limits = plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)
    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": edge_color}}}

    step_start = step_start or reach_interface.step_start
    step_end = step_end or reach_interface.step_end
    if steps:
        steps = [step for step in steps if step <= step_end + 1]

    else:
        steps = range(step_start, step_end + 1)
    duration = duration if duration else config.planning.dt

    message = "* Plotting reachable sets..."
    print(message)
    logger.info(message)

    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None
    for step in steps:
        time_step = step * int(config.planning.dt * 10)
        if config.debug.save_plots:
            # clear previous plot
            plt.cla()
        else:
            # create new figure
            plt.figure(figsize=figsize)
            renderer = MPRenderer(plot_limits=plot_limits)

        # plot scenario and planning problem
        scenario.draw(renderer, draw_params={"dynamic_obstacle": {"draw_icon": config.debug.draw_icons},
                                             "trajectory": {"draw_trajectory": True},
                                             "time_begin": time_step})
        if config.debug.draw_planning_problem:
            planning_problem.draw(renderer, draw_params={'planning_problem': {'initial_state': {'state': {
                'draw_arrow': False, "radius": 0.5}}}})

        list_nodes = reach_interface.reachable_set_at_step(step)
        draw_reachable_sets(list_nodes, config, renderer, draw_params)

        # settings and adjustments
        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        ax.set_xlabel(f"$s$ [m]", fontsize=28)
        ax.set_ylabel("$d$ [m]", fontsize=28)
        plt.margins(0, 0)
        renderer.render()

        if config.debug.draw_ref_path and ref_path is not None:
            renderer.ax.plot(ref_path[:, 0], ref_path[:, 1],
                             color='g', marker='.', markersize=1, zorder=19, linewidth=2.0)

        if config.debug.save_plots:
            save_fig(save_gif, path_output, step)
        else:
            plt.show()

    if config.debug.save_plots and save_gif:
        make_gif(path_output, "reachset_", steps, str(scenario.scenario_id), duration)

    message = "\tReachable sets plotted."
    print(message)
    logger.info(message)


def plot_scenario_with_drivable_area(reach_interface: ReachableSetInterface, figsize: Tuple = None,
                                     step_start: int = 0, step_end: int = 0, steps: List[int] = None,
                                     plot_limits: Union[List] = None, path_output: str = None,
                                     save_gif: bool = True, duration: float = None):
    config = reach_interface.config
    scenario = config.scenario
    planning_problem = config.planning_problem
    ref_path = config.planning.reference_path

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    figsize = figsize if figsize else (25, 15)
    plot_limits = plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)
    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": edge_color}}}

    step_start = step_start or reach_interface.step_start
    step_end = step_end or reach_interface.step_end
    if steps:
        steps = [step for step in steps if step <= step_end + 1]

    else:
        steps = range(step_start, step_end + 1)
    duration = duration if duration else config.planning.dt

    message = "* Plotting drivable area..."
    print(message)
    logger.info(message)

    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None
    for step in steps:
        time_step = step * int(config.planning.dt * 10)
        if config.debug.save_plots:
            # clear previous plot
            plt.cla()
        else:
            # create new figure
            plt.figure(figsize=figsize)
            renderer = MPRenderer(plot_limits=plot_limits)

        # plot scenario and planning problem
        scenario.draw(renderer, draw_params={"dynamic_obstacle": {"draw_icon": config.debug.draw_icons},
                                             "trajectory": {"draw_trajectory": True},
                                             "time_begin": time_step})
        if config.debug.draw_planning_problem:
            planning_problem.draw(renderer, draw_params={'planning_problem': {'initial_state': {'state': {
                'draw_arrow': False, "radius": 0.5}}}})

        list_nodes = reach_interface.drivable_area_at_step(step)
        draw_drivable_area(list_nodes, config, renderer, draw_params)

        # settings and adjustments
        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        ax.set_xlabel(f"$s$ [m]", fontsize=28)
        ax.set_ylabel("$d$ [m]", fontsize=28)
        plt.margins(0, 0)
        renderer.render()

        if config.debug.draw_ref_path and ref_path is not None:
            renderer.ax.plot(ref_path[:, 0], ref_path[:, 1],
                             color='g', marker='.', markersize=1, zorder=19, linewidth=2.0)

        if config.debug.save_plots:
            save_fig(save_gif, path_output, time_step)
        else:
            plt.show()

    if config.debug.save_plots and save_gif:
        make_gif(path_output, "reachset_", steps, str(scenario.scenario_id), duration=duration)

    message = "\tDrivable area plotted."
    print(message)
    logger.info(message)


def compute_plot_limits_from_reachable_sets(reach_interface: ReachableSetInterface, margin: int = 20):
    config = reach_interface.config
    x_min = y_min = np.infty
    x_max = y_max = -np.infty
    backend = "CPP" if config.reachable_set.mode_computation == 2 else "PYTHON"
    coordinate_system = config.planning.coordinate_system

    if coordinate_system == "CART":
        for step in range(reach_interface.step_start, reach_interface.step_end):
            for rectangle in reach_interface.drivable_area_at_step(step):
                bounds = rectangle.bounds if backend == "PYTHON" else (rectangle.p_lon_min(), rectangle.p_lat_min(),
                                                                       rectangle.p_lon_max(), rectangle.p_lat_max())
                x_min = min(x_min, bounds[0])
                y_min = min(y_min, bounds[1])
                x_max = max(x_max, bounds[2])
                y_max = max(y_max, bounds[3])

    elif config.planning.coordinate_system == "CVLN":
        for step in range(reach_interface.step_start, reach_interface.step_end):
            for rectangle_cvln in reach_interface.drivable_area_at_step(step):
                list_rectangles_cart = util_coordinate_system.convert_to_cartesian_polygons(rectangle_cvln,
                                                                                            config.planning.CLCS, False)
                for rectangle_cart in list_rectangles_cart:
                    bounds = rectangle_cart.bounds
                    x_min = min(x_min, bounds[0])
                    y_min = min(y_min, bounds[1])
                    x_max = max(x_max, bounds[2])
                    y_max = max(y_max, bounds[3])

    if np.inf in (x_min, y_min) or -np.inf in (x_max, y_max):
        return None

    else:
        return [x_min - margin, x_max + margin, y_min - margin, y_max + margin]


def draw_reachable_sets(list_nodes, config, renderer, draw_params):
    backend = "CPP" if config.reachable_set.mode_computation == 2 else "PYTHON"
    coordinate_system = config.planning.coordinate_system

    if coordinate_system == "CART":
        for node in list_nodes:
            vertices = node.position_rectangle.vertices if backend == "PYTHON" else node.position_rectangle().vertices()
            Polygon(vertices=np.array(vertices)).draw(renderer, draw_params=draw_params)

    elif coordinate_system == "CVLN":
        for node in list_nodes:
            position_rectangle = node.position_rectangle if backend == "PYTHON" else node.position_rectangle()
            list_polygons_cart = util_coordinate_system.convert_to_cartesian_polygons(position_rectangle,
                                                                                      config.planning.CLCS, True)
            for polygon in list_polygons_cart:
                Polygon(vertices=np.array(polygon.vertices)).draw(renderer, draw_params=draw_params)


def draw_drivable_area(list_rectangles, config, renderer, draw_params):
    backend = "CPP" if config.reachable_set.mode_computation == 2 else "PYTHON"
    coordinate_system = config.planning.coordinate_system

    if coordinate_system == "CART":
        for rect in list_rectangles:
            vertices = rect.vertices if backend == "PYTHON" else rect.vertices()
            Polygon(vertices=np.array(vertices)).draw(renderer, draw_params=draw_params)

    elif coordinate_system == "CVLN":
        for rect in list_rectangles:
            list_polygons_cart = util_coordinate_system.convert_to_cartesian_polygons(rect, config.planning.CLCS, True)
            for polygon in list_polygons_cart:
                Polygon(vertices=np.array(polygon.vertices)).draw(renderer, draw_params=draw_params)


def save_fig(save_gif: bool, path_output: str, time_step: int):
    if save_gif:
        # save as png
        print("\tSaving", os.path.join(path_output, f'{"reachset"}_{time_step:05d}.png'))
        plt.savefig(os.path.join(path_output, f'{"reachset"}_{time_step:05d}.png'), format="png", bbox_inches="tight",
                    transparent=False)

    else:
        # save as svg
        print("\tSaving", os.path.join(path_output, f'{"reachset"}_{time_step:05d}.svg'))
        plt.savefig(f'{path_output}{"reachset"}_{time_step:05d}.svg', format="svg", bbox_inches="tight",
                    transparent=False)


def make_gif(path: str, prefix: str, steps: Union[range, List[int]],
             file_save_name="animation", duration: float = 0.1):
    images = []
    filenames = []

    print("\tCreating GIF...")
    for step in steps:
        im_path = os.path.join(path, prefix + "{:05d}.png".format(step))
        filenames.append(im_path)

    for filename in filenames:
        images.append(imageio.imread(filename))

    imageio.mimsave(os.path.join(path, "../", file_save_name + ".gif"), images, duration=duration)
    print("\tGIF created.")


def plot_scenario_with_driving_corridor(driving_corridor, dc_id: int, reach_interface: ReachableSetInterface,
                                        step_start: Union[int, None] = None, step_end: Union[int, None] = None,
                                        steps: Union[List[int], None] = None, save_gif: bool = False,
                                        duration: float = None, as_svg=False):
    """2D visualization of a given driving corridor and scenario.

    :param driving_corridor: Driving corridor to visualize
    :param dc_id: id of driving corridor (idx in DC list)
    :param reach_interface: ReachableSetInterface object
    :param step_start: start time step for plotting
    :param step_end: end time step
    :param steps: list of steps to plot
    :param save_gif: make gif (works only if step_end is given)
    :param as_svg: save figures as svg for nice paper plots
    """
    # set ups
    config = reach_interface.config
    scenario = config.scenario

    planning_problem = config.planning_problem
    ref_path = config.planning.reference_path

    # set color
    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": edge_color}}}

    # set step_start and step_end
    step_start = step_start or reach_interface.step_start
    step_end = step_end or reach_interface.step_end

    # Check validity of specified time step fo visualization
    assert step_end >= step_start, \
        "Specified end time step for visualization has to greater than start time step"
    assert step_end in range(reach_interface.step_end + 1), \
        "Specified end time step for visualization is too high."

    if steps:
        steps = [step for step in steps if step <= step_end + 1]
    else:
        steps = range(step_start, step_end + 1)
    duration = duration if duration else config.planning.dt

    message = ("* Plotting driving corridor no. %s ..." % dc_id)
    print(message)
    logger.info(message)

    # create output directory
    path_output = config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    # set plot limits & create renderer
    plot_limits = config.debug.plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)
    renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))

    # create separate output folder
    path_output_lon_dc = path_output + ('lon_driving_corridor_%s/' % dc_id)
    Path(path_output_lon_dc).mkdir(parents=True, exist_ok=True)

    # make separate plot of driving corridor for each time step + create gif (optional)
    for step in steps:
        time_step = step * round(config.planning.dt * 10)
        # plot driving corridor and scenario at the specified time step
        plt.cla()
        scenario.draw(renderer, draw_params={"dynamic_obstacle": {"draw_icon": True}, "time_begin": time_step})
        # draw planning problem
        if config.debug.draw_planning_problem:
            planning_problem.draw(renderer, draw_params={'planning_problem': {'initial_state': {'state': {
                'draw_arrow': False, "radius": 0.5}}}})
        # reach set nodes in driving corridor at specified time step
        list_nodes = driving_corridor[step]
        draw_reachable_sets(list_nodes, config, renderer, draw_params)

        # plot
        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        plt.margins(0, 0)
        renderer.render()

        # draw reference path
        if config.debug.draw_ref_path and ref_path is not None:
            renderer.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19,
                             linewidth=1.5)

        if config.debug.save_plots:
            save_format = "svg" if as_svg else "png"
            print("\tSaving", os.path.join(path_output_lon_dc, f'{"lon_driving_corridor"}_{time_step:05d}.{save_format}'))
            plt.savefig(
                f'{path_output_lon_dc}{"lon_driving_corridor"}_{time_step:05d}.{save_format}',
                format=save_format, bbox_inches="tight", transparent=False)

    if config.debug.save_plots and save_gif:
        make_gif(path_output_lon_dc, "lon_driving_corridor_", steps, ("lon_driving_corridor_%s" % dc_id), duration)

    message = ("\tDriving corridor %s plotted." % dc_id)
    print(message)
    logger.info(message)


def draw_driving_corridor_3d(driving_corridor: Dict, dc_id, reach_interface: ReachableSetInterface,
                             lanelet_ids: List[int] = None, list_obstacles: List = None, as_svg=False):
    """Draws a driving corridor with 3D projection."""
    message = "* Plotting 3D driving corridor ..."
    print(message)
    logger.info(message)

    # get settings from config
    config = reach_interface.config
    lanelet_network = create_lanelet_network_from_ids(config.scenario.lanelet_network, lanelet_ids) \
        if lanelet_ids else config.scenario.lanelet_network

    # temporal length of driving corridor
    step_end = len(driving_corridor) - 1

    # setup figure
    fig = plt.figure(figsize=(20, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.computed_zorder = False
    palette = sns.color_palette("GnBu_d", 3)
    # set plot limits
    plot_limits = config.debug.plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)

    # create output directory
    path_output = config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    # settings for 3d view
    interval = 0.04
    height = 0.04

    # 3D rendering of lanelet network
    _render_lanelet_network_3d(lanelet_network, ax)

    for step in range(step_end + 1):
        time_step = step * round(config.planning.dt * 10)
        # determine tuple (z_min, z_max) for polyhedron
        z_tuple = (step * interval, step * interval + height)

        # 3D rendering of obstacles
        if list_obstacles:
            for obs in list_obstacles:
                occ_rectangle = obs.occupancy_at_time(time_step).shape
                _render_obstacle_3d(occ_rectangle, ax, z_tuple)

        # 3D rendering of reachable sets
        list_reach_nodes = driving_corridor[step]
        _render_reachable_sets_3d(list_reach_nodes, ax, z_tuple, config, palette)

    # axis settings
    ax.set_xlim(plot_limits[0:2])
    ax.set_ylim(plot_limits[2:4])
    ax.set_zlim([0, 3.5])
    # ax.set_xticks([])
    # ax.set_yticks([])
    # ax.set_zticks([])
    ax.set_axis_off()
    ax.view_init(azim=config.debug.plot_azimuth, elev=config.debug.plot_elevation)
    ax.dist = config.debug.ax_distance

    if config.debug.save_plots:
        save_format = "svg" if as_svg else "png"
        plt.savefig(f'{path_output}{"lon_driving_corridor"}_{dc_id}_3D.{save_format}', format=save_format,
                    bbox_inches='tight', transparent=False)
    plt.show()

    message = "\t3D driving corridor plotted."
    print(message)
    logger.info(message)


def _render_reachable_sets_3d(list_reach_nodes, ax, z_tuple, config, palette):
    """
    Renders a 3d visualization of a given list of reach set nodes onto the provided axis object
    """
    # get information from config
    coordinate_system = config.planning.coordinate_system
    backend = "CPP" if config.reachable_set.mode_computation == 2 else "PYTHON"

    # set colors
    face_color = palette[0]
    edge_color = (palette[0][0] * 0.30, palette[0][1] * 0.30, palette[0][2] * 0.30)

    if coordinate_system == "CART":
        for node in list_reach_nodes:
            pos_rectangle = node.position_rectangle if backend == "PYTHON" else node.position_rectangle()
            z_values, vertices_3d = _compute_vertices_of_polyhedron(pos_rectangle, z_tuple)
            # render in 3D
            ax.scatter3D(z_values[:, 0], z_values[:, 1], z_values[:, 2], alpha=0.0)
            pc = Poly3DCollection(vertices_3d)
            pc.set_facecolor(face_color)
            pc.set_edgecolor(edge_color)
            pc.set_linewidth(0.8)
            pc.set_alpha(0.9)
            pc.set_zorder(30)
            ax.add_collection3d(pc)

    elif coordinate_system == "CVLN":
        for node in list_reach_nodes:
            pos_rectangle_cvln = node.position_rectangle if backend == "PYTHON" else node.position_rectangle()
            list_pos_rectangle_cart = util_coordinate_system.convert_to_cartesian_polygons(pos_rectangle_cvln,
                                                                                           config.planning.CLCS, True)
            for pos_rectangle in list_pos_rectangle_cart:
                z_values, vertices_3d = _compute_vertices_of_polyhedron(pos_rectangle, z_tuple)
                # render in 3D
                ax.scatter3D(z_values[:, 0], z_values[:, 1], z_values[:, 2], alpha=0.0)
                pc = Poly3DCollection(vertices_3d)
                pc.set_facecolor(face_color)
                pc.set_edgecolor(edge_color)
                pc.set_linewidth(0.8)
                pc.set_alpha(0.9)
                pc.set_zorder(30)
                ax.add_collection3d(pc)


def _compute_vertices_of_polyhedron(position_rectangle, z_tuple):
    """
    Computes vertices of Polyhedron given a 2D position rectangle and z coordinates as tuple (z_min, z_max)
    """
    # unpack z tuple
    z_min = z_tuple[0]
    z_max = z_tuple[1]

    # get vertices of position rectangle
    if isinstance(position_rectangle, ReachPolygon):
        # ReachPolygon Python data structure
        x_min = position_rectangle.p_lon_min
        x_max = position_rectangle.p_lon_max
        y_min = position_rectangle.p_lat_min
        y_max = position_rectangle.p_lat_max
    else:
        # ReachPolygon CPP data structure
        x_min = position_rectangle.p_lon_min()
        x_max = position_rectangle.p_lon_max()
        y_min = position_rectangle.p_lat_min()
        y_max = position_rectangle.p_lat_max()

    # z values for 3D visualization of reachable sets as polyhedron
    z_values = np.array([[x_min, y_min, z_min],
                         [x_max, y_min, z_min],
                         [x_max, y_max, z_min],
                         [x_min, y_max, z_min],
                         [x_min, y_min, z_max],
                         [x_max, y_min, z_max],
                         [x_max, y_max, z_max],
                         [x_min, y_max, z_max]])

    # vertices of the 6 surfaces of the polyhedron
    vertices = [[z_values[0], z_values[1], z_values[2], z_values[3]],
                [z_values[4], z_values[5], z_values[6], z_values[7]],
                [z_values[0], z_values[1], z_values[5], z_values[4]],
                [z_values[2], z_values[3], z_values[7], z_values[6]],
                [z_values[1], z_values[2], z_values[6], z_values[5]],
                [z_values[4], z_values[7], z_values[3], z_values[0]]]

    return z_values, vertices


def _render_lanelet_network_3d(lanelet_network, ax):
    """
    Renders a 3d visualization of a given lanelet network onto the provided axis object
    """
    for lanelet in lanelet_network.lanelets:
        x = np.array(lanelet.left_vertices[:, 0].tolist() + np.flip(lanelet.right_vertices[:, 0]).tolist())
        y = np.array(lanelet.left_vertices[:, 1].tolist() + np.flip(lanelet.right_vertices[:, 1]).tolist()) - 0
        # z = np.zeros(x.shape)

        # ax.scatter3D(x, y, z, alpha=0.0)
        # vertices = [list(zip(x, y, z))]
        vertices = [list(zip(x, y))]
        pc = PolyCollection(vertices)
        pc.set_facecolor("#c7c7c7")
        pc.set_edgecolor("#dddddd")
        pc.set_linewidth(0.5)
        pc.set_alpha(0.7)
        pc.set_zorder(0)
        ax.add_collection3d(pc, zs=0, zdir='z')


def _render_obstacle_3d(occupancy_rect: Rectangle, ax, z_tuple: Tuple):
    """
    Renders a 3d visualization of a CR obstacle occupancy (given as a CommonRoad Rectangle shape)
    """
    assert isinstance(occupancy_rect, Rectangle)
    # unpack z tuple
    z_min = z_tuple[0]
    z_max = z_tuple[1]

    # get 2D vertices of rectangle
    vertices_2d = occupancy_rect.vertices[:-1].tolist()

    # z values for 3D visualization
    z_values = []
    tmp1 = deepcopy(vertices_2d)
    for i in range(len(vertices_2d)):
        tmp1[i].append(z_min)
        z_values.append(tmp1[i])
    tmp2 = deepcopy(vertices_2d)
    for i in range(len(vertices_2d)):
        tmp2[i].append(z_max)
        z_values.append(tmp2[i])

    z_values = np.array(z_values)

    vertices = [[z_values[0], z_values[1], z_values[2], z_values[3]],
                [z_values[4], z_values[5], z_values[6], z_values[7]],
                [z_values[0], z_values[1], z_values[5], z_values[4]],
                [z_values[2], z_values[3], z_values[7], z_values[6]],
                [z_values[1], z_values[2], z_values[6], z_values[5]],
                [z_values[4], z_values[7], z_values[3], z_values[0]]]

    ax.scatter3D(z_values[:, 0], z_values[:, 1], z_values[:, 2], alpha=0.0)
    pc = Poly3DCollection(vertices)
    pc.set_facecolor("#1d7eea")
    pc.set_edgecolor("#00478f")
    pc.set_linewidth(0.5)
    pc.set_alpha(0.9)
    pc.set_zorder(20)
    ax.add_collection3d(pc)
