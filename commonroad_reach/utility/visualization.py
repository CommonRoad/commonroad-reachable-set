import logging
import os
from copy import deepcopy
from pathlib import Path
from typing import Tuple, Union, List, Optional

import imageio
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from commonroad.geometry.shape import Polygon, Rectangle, Circle
from commonroad.visualization.draw_params import MPDrawParams, ShapeParams
from commonroad.visualization.mp_renderer import MPRenderer
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, PolyCollection

from commonroad_clcs import pycrccosy

import commonroad_reach.utility.logger as util_logger
from commonroad_reach import pycrreach
from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.driving_corridor import DrivingCorridor
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import coordinate_system as util_coordinate_system
from commonroad_reach.utility.general import create_lanelet_network_from_ids
from commonroad_reach.utility.configuration import compute_disc_radius_and_distance

logger = logging.getLogger("REACH_LOGGER")
logging.getLogger('PIL').setLevel(logging.WARNING)
logging.getLogger('matplotlib.font_manager').setLevel(logging.WARNING)


# ***************
# color settings
# ***************
REACH_COLOR_PALETTE = sns.color_palette("GnBu_d", 3)
REACH_COLOR_SETTINGS: dict = {
    "facecolor": REACH_COLOR_PALETTE[0],
    "edgecolor": (REACH_COLOR_PALETTE[0][0] * 0.75, REACH_COLOR_PALETTE[0][1] * 0.75, REACH_COLOR_PALETTE[0][2] * 0.75)
}


def plot_scenario_with_reachable_sets(reach_interface: ReachableSetInterface, figsize: Tuple = None,
                                      step_start: int = 0, step_end: int = 0, steps: List[int] = None,
                                      plot_limits: List = None, path_output: str = None,
                                      save_gif: bool = True, duration: float = None, terminal_set=None):
    """
    Plots scenario with computed reachable sets.
    """
    config = reach_interface.config
    scenario = config.scenario
    planning_problem = config.planning_problem
    ref_path = config.planning.reference_path

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    figsize = figsize if figsize else (25, 15)
    plot_limits = plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)

    # generate default drawing parameters
    draw_params = generate_default_drawing_parameters(config)

    step_start = step_start or reach_interface.step_start
    step_end = step_end or reach_interface.step_end
    if steps:
        steps = [step for step in steps if step <= step_end + 1]
    else:
        steps = range(step_start, step_end + 1)
    duration = duration if duration else config.planning.dt

    util_logger.print_and_log_info(logger, "* Plotting reachable sets...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None
    for step in steps:
        time_step = step * round(config.planning.dt / config.scenario.dt)
        if config.debug.save_plots:
            # clear previous plot
            plt.cla()
        else:
            # create new figure
            plt.figure(figsize=figsize)
            renderer = MPRenderer(plot_limits=plot_limits)

        # plot scenario and planning problem
        draw_params.time_begin = time_step
        scenario.draw(renderer, draw_params)

        if config.debug.draw_planning_problem:
            planning_problem.draw(renderer, draw_params)

        list_nodes = reach_interface.reachable_set_at_step(step)
        draw_reachable_sets(list_nodes, config, renderer, draw_params)

        # plot terminal set
        if terminal_set:
            draw_params_temp = MPDrawParams()
            draw_params_temp.shape.opacity = 1.0
            draw_params_temp.shape.linewidth = 0.5
            draw_params_temp.shape.facecolor = "#f1b514"
            draw_params_temp.shape.edgecolor = "#302404"
            draw_params_temp.shape.zorder = 15

            terminal_set.draw(renderer, draw_params_temp)

        # settings and adjustments
        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        ax.set_xlabel(f"$s$ [m]", fontsize=28)
        ax.set_ylabel("$d$ [m]", fontsize=28)
        plt.margins(0, 0)
        renderer.render()

        # plot reference path
        if config.debug.draw_ref_path and ref_path is not None:
            renderer.ax.plot(ref_path[:, 0], ref_path[:, 1],
                             color='g', marker='.', markersize=1, zorder=19, linewidth=2.0)

        if config.debug.save_plots:
            save_fig(save_gif, path_output, step, verbose=(step % 5 == 0))
        else:
            plt.show()

    if config.debug.save_plots and save_gif:
        make_gif(path_output, "png_reach_", steps, str(scenario.scenario_id), duration)

    util_logger.print_and_log_info(logger, "\tReachable sets plotted.")


def generate_default_drawing_parameters(config: Configuration) -> MPDrawParams:
    """Creates default drawing parameters for visualization functions"""
    draw_params = MPDrawParams()

    # default settings for CR scenario
    draw_params.dynamic_obstacle.draw_icon = config.debug.draw_icons
    draw_params.dynamic_obstacle.trajectory.draw_trajectory = True
    draw_params.dynamic_obstacle.occupancy.draw_occupancies = False
    draw_params.lanelet_network.lanelet.show_label = config.debug.draw_lanelet_labels
    draw_params.planning_problem.initial_state.state.draw_arrow = False
    draw_params.planning_problem.initial_state.state.radius = 0.5

    # default face color and edge color of reach polygons
    draw_params.shape.facecolor = REACH_COLOR_SETTINGS["facecolor"]
    draw_params.shape.edgecolor = REACH_COLOR_SETTINGS["edgecolor"]

    return draw_params


def compute_plot_limits_from_reachable_sets(reach_interface: ReachableSetInterface, margin: int = 20):
    """
    Returns plot limits from the computed reachable sets.

    :param reach_interface: interface holding the computed reachable sets.
    :param margin: additional margin for the plot limits.
    :return:
    """
    config = reach_interface.config
    x_min = y_min = np.inf
    x_max = y_max = -np.inf
    coordinate_system = config.planning.coordinate_system

    if coordinate_system == "CART":
        for step in range(reach_interface.step_start, reach_interface.step_end):
            for rectangle in reach_interface.drivable_area_at_step(step):
                bounds = rectangle.bounds
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


def draw_reachable_sets_cart(
    list_nodes: List,
    renderer: MPRenderer,
    draw_params: Optional[MPDrawParams] = None
) -> None:
    """Draws Cartesian reachable sets in Cartesian coordinates"""
    # use default colors if draw params not given
    if draw_params is None:
        draw_params = MPDrawParams()
        draw_params.shape.facecolor = REACH_COLOR_SETTINGS["facecolor"]
        draw_params.shape.edgecolor = REACH_COLOR_SETTINGS["edgecolor"]

    # draw reachable sets as CR polygons
    for node in list_nodes:
        vertices = node.position_rectangle.vertices
        Polygon(vertices=np.array(vertices)).draw(renderer, draw_params)


def draw_reachable_sets_cvln(
    list_nodes: List,
    renderer: MPRenderer,
    clcs: pycrccosy.CurvilinearCoordinateSystem,
    draw_params: Optional[MPDrawParams] = None
) -> None:
    """Draws Curvilinear reachable sets in Cartesian coordinates. Sets are transformed before drawing."""
    # use default colors if draw params not given
    if draw_params is None:
        draw_params = MPDrawParams()
        draw_params.shape.facecolor = REACH_COLOR_SETTINGS["facecolor"]
        draw_params.shape.edgecolor = REACH_COLOR_SETTINGS["edgecolor"]

    # transforms CLVN polygons to CART and draw reachable sets as CR polygons
    for node in list_nodes:
        position_rectangle = node.position_rectangle
        list_polygons_cart = util_coordinate_system.convert_to_cartesian_polygons(
            position_rectangle,
            clcs,
            True
        )
        for polygon in list_polygons_cart:
            Polygon(vertices=np.array(polygon.vertices)).draw(renderer, draw_params)


def draw_reachable_sets(
    list_nodes: List,
    config: Configuration,
    renderer: MPRenderer,
    draw_params: MPDrawParams
):
    """Draw a list of reachable sets on the given renderer object"""
    coordinate_system = config.planning.coordinate_system

    if coordinate_system == "CART":
        draw_reachable_sets_cart(list_nodes, renderer, draw_params)

    elif coordinate_system == "CVLN":
        draw_reachable_sets_cvln(list_nodes, renderer, config.planning.CLCS, draw_params)


def save_fig(save_gif: bool, path_output: str, time_step: int, identifier: str = "reach", verbose: bool = True):
    if save_gif:
        # save as png
        name_figure = "png_" + identifier
        path_figure = os.path.join(path_output, f'{name_figure}_{time_step:05d}.png')
        plt.savefig(path_figure, format="png", bbox_inches="tight", transparent=False)

    else:
        # save as svg
        name_figure = "svg" + identifier
        path_figure = os.path.join(path_output, f'{name_figure}_{time_step:05d}.svg')
        plt.savefig(path_figure, format="svg", bbox_inches="tight", transparent=False)

    if verbose:
        util_logger.print_and_log_info(logger, f"\tSaving {path_figure}")


def make_gif(path: str, prefix: str, steps: Union[range, List[int]],
             file_save_name="animation", duration: float = 0.1):
    images = []
    filenames = []

    util_logger.print_and_log_info(logger, "\tCreating GIF...")
    for step in steps:
        im_path = os.path.join(path, prefix + "{:05d}.png".format(step))
        filenames.append(im_path)

    for filename in filenames:
        images.append(imageio.imread(filename))

    imageio.mimsave(os.path.join(path, "../", file_save_name + ".gif"), images, duration=duration)


def plot_scenario_with_driving_corridor(driving_corridor: DrivingCorridor, dc_id: int,
                                        reach_interface: ReachableSetInterface,
                                        step_start: Union[int, None] = None, step_end: Union[int, None] = None,
                                        steps: Union[List[int], None] = None, save_gif: bool = False,
                                        duration: float = None, as_svg=False, terminal_set=None):
    """
    2D visualization of a given driving corridor and scenario.

    :param driving_corridor: Driving corridor to visualize
    :param dc_id: id of driving corridor (idx in DC list)
    :param reach_interface: ReachableSetInterface object
    :param step_start: start step for plotting
    :param step_end: end step
    :param steps: list of steps to plot
    :param save_gif: make gif (works only if step_end is given)
    :param as_svg: save figures as svg for nice paper plots
    :param duration: duration of a step
    :param terminal_set: terminal set at which the driving corridor should end
    """
    config = reach_interface.config
    scenario = config.scenario
    planning_problem = config.planning_problem
    ref_path = config.planning.reference_path

    path_output = config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)
    # create separate output folder for corridor
    path_output_lon_dc = path_output + ('driving_corridor_%s/' % dc_id)
    Path(path_output_lon_dc).mkdir(parents=True, exist_ok=True)

    figsize = (25, 15)
    plot_limits = config.debug.plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)

    # generate default drawing parameters
    draw_params = generate_default_drawing_parameters(config)

    step_start = step_start or reach_interface.step_start
    step_end = step_end or reach_interface.step_end
    assert step_end >= step_start, "Specified end step for visualization has to greater than start step"
    assert step_end in range(reach_interface.step_end + 1), "Specified end step for visualization is too large."
    if steps:
        steps = [step for step in steps if step <= step_end + 1]
    else:
        steps = range(step_start, step_end + 1)
    duration = duration if duration else config.planning.dt

    util_logger.print_and_log_info(logger, f"* Plotting driving corridor #{dc_id} ...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None
    # make separate plot of driving corridor for each step + create gif (optional)
    for step in steps:
        time_step = step * round(config.planning.dt / config.scenario.dt)
        # plot driving corridor and scenario at the specified time step
        plt.cla()

        draw_params.time_begin = time_step
        scenario.draw(renderer, draw_params)
        # draw planning problem
        if config.debug.draw_planning_problem:
            planning_problem.draw(renderer, draw_params)

        # reach set nodes in driving corridor at specified step
        list_nodes = driving_corridor.reach_nodes_at_step(step)
        draw_reachable_sets(list_nodes, config, renderer, draw_params)

        # draw terminal_set
        if terminal_set is not None:
            from commonroad.geometry.shape import Polygon
            # convert terminal_set to Cartesian
            CLCS = config.planning.CLCS
            # ts_x, ts_y = terminal_set.shapely_object.exterior.coords.xy
            # terminal_set_vertices = [vertex for vertex in zip(ts_x, ts_y)]
            transformed_rectangle, triangle_mesh = CLCS.convert_rectangle_to_cartesian_coords(
                terminal_set[0], terminal_set[1], terminal_set[2], terminal_set[3])  #
            # create CommonRoad Polygon
            terminal_shape = Polygon(vertices=np.array(transformed_rectangle))

            draw_params_temp = MPDrawParams()
            draw_params_temp.shape.opacity = 1.0
            draw_params_temp.shape.linewidth = 0.5
            draw_params_temp.shape.facecolor = "#f1b514"
            draw_params_temp.shape.edgecolor = "#302404"
            draw_params_temp.shape.zorder = 15
            terminal_shape.draw(renderer, draw_params_temp)

        # settings and adjustments
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
            if step % 5 == 0:
                print("\tSaving",
                      os.path.join(path_output_lon_dc, f'{"driving_corridor"}_{time_step:05d}.{save_format}'))

            plt.savefig(
                f'{path_output_lon_dc}{"driving_corridor"}_{time_step:05d}.{save_format}',
                format=save_format, bbox_inches="tight", transparent=False)

    if config.debug.save_plots and save_gif:
        make_gif(path_output_lon_dc, "driving_corridor_", steps, ("driving_corridor_%s" % dc_id), duration)

    util_logger.print_and_log_info(logger, f"\tDriving corridor {dc_id} plotted.")


def draw_driving_corridor_2d(driving_corridor: DrivingCorridor, dc_id: int, reach_interface: ReachableSetInterface,
                             trajectory: np.ndarray = None, as_svg: bool = False, rnd: MPRenderer = None):
    """
    Draws full driving corridor in 2D and (optionally) visualizes planned trajectory within the corridor.
    """
    # set ups
    config = reach_interface.config
    scenario = config.scenario

    util_logger.print_and_log_info(logger, "* Plotting full 2D driving corridor ...", verbose=config.debug.verbose_info)

    planning_problem = config.planning_problem

    # generate default drawing parameters
    draw_params = generate_default_drawing_parameters(config)

    # create output directory
    path_output = config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    # set plot limits & create renderer
    if rnd is None:
        plot_limits = config.debug.plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)
        renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))
        plt.cla()
    else:
        renderer = rnd

    # draw scenario at first step
    draw_params.time_begin = 0
    scenario.draw(renderer, draw_params)

    # draw planning problem
    if config.debug.draw_planning_problem:
        planning_problem.draw(renderer, draw_params)

    # plot full driving corridor (for all steps)
    for step in driving_corridor.dict_step_to_cc.keys():
        # reach set nodes in driving corridor at specified step
        list_nodes = driving_corridor.reach_nodes_at_step(step)
        draw_reachable_sets(list_nodes, config, renderer, draw_params)

    # plot
    plt.rc("axes", axisbelow=True)
    ax = plt.gca()
    ax.set_aspect("equal")
    plt.margins(0, 0)

    if rnd is None:
        renderer.render()

    if trajectory is not None:
        renderer.ax.plot(trajectory[:, 0], trajectory[:, 1],
                         color='k', marker='.', markersize=10, zorder=50, linewidth=3.0)

    if config.debug.save_plots:
        save_format = "svg" if as_svg else "png"
        plt.savefig(
            f'{path_output}{"driving_corridor"}_{dc_id}_2D.{save_format}', format=save_format,
            bbox_inches="tight", transparent=False, dpi=300)


def draw_driving_corridor_3d(driving_corridor: DrivingCorridor, dc_id: int, reach_interface: ReachableSetInterface,
                             list_obstacle_ids: List[int] = None, as_svg: bool = False):
    """
    Draws full driving corridor with 3D projection.
    """
    util_logger.print_and_log_info(logger, "* Plotting full 3D driving corridor ...")

    # get settings from config
    config = reach_interface.config

    # temporal length of driving corridor
    step_end = driving_corridor.step_final

    # setup figure
    fig = plt.figure(figsize=(20, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.computed_zorder = False
    palette = sns.color_palette("GnBu_d", 3)
    # set plot limits
    plot_limits = config.debug.plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)

    # get relevant lanelet IDs from plot limits
    plot_window_length, plot_window_width = plot_limits[1] - plot_limits[0], plot_limits[3] - plot_limits[2]
    center = np.array([plot_limits[0] + plot_window_length / 2, plot_limits[2] + plot_window_width / 2])
    rect = Rectangle(plot_window_length, plot_window_width, center)
    lanelet_ids = config.scenario.lanelet_network.find_lanelet_by_shape(rect)

    # create lanelet network for plotting
    lanelet_network = create_lanelet_network_from_ids(config.scenario.lanelet_network, lanelet_ids) \
        if lanelet_ids else config.scenario.lanelet_network

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
        if list_obstacle_ids:
            for obs_id in list_obstacle_ids:
                obs = config.scenario.obstacle_by_id(obs_id)
                occ_rectangle = obs.occupancy_at_time(time_step).shape
                _render_obstacle_3d(occ_rectangle, ax, z_tuple)

        # 3D rendering of reachable sets
        list_reach_nodes = driving_corridor.reach_nodes_at_step(step)
        _render_reachable_sets_3d(list_reach_nodes, ax, z_tuple, config, palette)

    # axis settings
    ax.set_xlim(plot_limits[0:2])
    ax.set_ylim(plot_limits[2:4])
    ax.set_zlim([0, 3.5])
    ax.set_axis_off()
    ax.view_init(azim=config.debug.plot_azimuth, elev=config.debug.plot_elevation)
    ax.dist = config.debug.ax_distance

    if config.debug.save_plots:
        save_format = "svg" if as_svg else "png"
        plt.savefig(f'{path_output}{"driving_corridor"}_{dc_id}_3D.{save_format}', format=save_format,
                    bbox_inches='tight', transparent=False)

    util_logger.print_and_log_info(logger, "\t3D driving corridor plotted.")


def _render_reachable_sets_3d(list_reach_nodes, ax, z_tuple, config, palette):
    """
    Renders a 3d visualization of a given list of reach nodes onto the provided axis object.
    """
    # get information from config
    coordinate_system = config.planning.coordinate_system

    # set colors
    face_color = palette[0]
    edge_color = (palette[0][0] * 0.30, palette[0][1] * 0.30, palette[0][2] * 0.30)

    if coordinate_system == "CART":
        for node in list_reach_nodes:
            pos_rectangle = node.position_rectangle
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
            pos_rectangle_cvln = node.position_rectangle
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
    Computes vertices of Polyhedron given a 2D position rectangle and z coordinates as a tuple (z_min, z_max).
    """
    # unpack z tuple
    z_min = z_tuple[0]
    z_max = z_tuple[1]

    # get vertices of position rectangle
    x_min = position_rectangle.p_lon_min
    x_max = position_rectangle.p_lon_max
    y_min = position_rectangle.p_lat_min
    y_max = position_rectangle.p_lat_max

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
    Renders a 3d visualization of a given lanelet network onto the provided axis object.
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
    Renders a 3d visualization of a CR obstacle occupancy (given as a CommonRoad Rectangle shape).
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


def plot_scenario_with_reachable_sets_cpp(reachable_set: pycrreach.ReachableSet, config: Configuration,
                                          step_start: int = 0, step_end: int = 0, steps: List[int] = None,
                                          plot_limits: List = None, figsize: Tuple = None, path_output: str = None,
                                          save_gif: bool = True, duration: float = None, terminal_set=None):
    """
    Plots scenario with computed reachable sets.

    Called by C++ script.
    """
    scenario = config.scenario
    planning_problem = config.planning_problem
    ref_path = config.planning.reference_path

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    figsize = figsize if figsize else (25, 15)
    plot_limits = plot_limits or compute_plot_limits_from_reachable_sets_cpp(reachable_set, config)

    # generate default drawing parameters
    draw_params = generate_default_drawing_parameters(config)

    step_start = step_start or reachable_set.step_start
    step_end = step_end or reachable_set.step_end
    if steps:
        steps = [step for step in steps if step <= step_end + 1]
    else:
        steps = range(step_start, step_end + 1)
    duration = duration if duration else config.planning.dt

    util_logger.print_and_log_info(logger, "* Plotting reachable sets...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=figsize) if config.debug.save_plots else None
    for step in steps:
        time_step = step * round(config.planning.dt / config.scenario.dt)
        if config.debug.save_plots:
            # clear previous plot
            plt.cla()
        else:
            # create new figure
            plt.figure(figsize=figsize)
            renderer = MPRenderer(plot_limits=plot_limits)

        # plot scenario and planning problem
        draw_params.time_begin = time_step
        scenario.draw(renderer, draw_params)

        if config.debug.draw_planning_problem:
            planning_problem.draw(renderer, draw_params)

        list_nodes = reachable_set.reachable_set_at_step(step)
        draw_reachable_sets(list_nodes, config, renderer, draw_params)

        # plot terminal set
        if terminal_set:
            draw_params_temp = MPDrawParams()
            draw_params_temp.shape.opacity = 1.0
            draw_params_temp.shape.linewidth = 0.5
            draw_params_temp.shape.facecolor = "#f1b514"
            draw_params_temp.shape.edgecolor = "#302404"
            draw_params_temp.shape.zorder = 15

            terminal_set.draw(renderer, draw_params_temp)

        # plot reference path
        if config.debug.draw_ref_path and ref_path is not None:
            renderer.ax.plot(ref_path[:, 0], ref_path[:, 1],
                             color='g', marker='.', markersize=1, zorder=19, linewidth=2.0)

        # settings and adjustments
        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        ax.set_xlabel(f"$s$ [m]", fontsize=28)
        ax.set_ylabel("$d$ [m]", fontsize=28)
        plt.margins(0, 0)
        renderer.render()

        if config.debug.save_plots:
            save_fig(save_gif, path_output, step, verbose=(step % 5 == 0))
        else:
            plt.show()

    if config.debug.save_plots and save_gif:
        make_gif(path_output, "png_reachset_", steps, str(scenario.scenario_id), duration)

    util_logger.print_and_log_info(logger, "\tReachable sets plotted.")


def compute_plot_limits_from_reachable_sets_cpp(reachable_set: pycrreach.ReachableSet,
                                                config: Configuration, margin: int = 20):
    """
    Returns plot limits from the computed reachable sets.

    :param reachable_set: C++ class holding the computed reachable sets.
    :param config: configuration file.
    :param margin: additional margin for the plot limits.
    :return:
    """
    x_min = y_min = np.inf
    x_max = y_max = -np.inf
    coordinate_system = config.planning.coordinate_system

    if coordinate_system == "CART":
        for step in range(reachable_set.step_start, reachable_set.step_end):
            for rectangle in reachable_set.drivable_area_at_step(step):
                bounds = rectangle.bounds
                x_min = min(x_min, bounds[0])
                y_min = min(y_min, bounds[1])
                x_max = max(x_max, bounds[2])
                y_max = max(y_max, bounds[3])

    elif coordinate_system == "CVLN":
        for step in range(reachable_set.step_start, reachable_set.step_end):
            for rectangle_cvln in reachable_set.drivable_area_at_step(step):
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


def plot_scenario_with_projection_domain(config: Configuration):
    """
    Plots scenario including projection domain of the curvilinear coordinate system used by reach_interface
    :param config: Configuration object of the ReachInterface
    """
    rnd = MPRenderer(figsize=(20, 10))
    draw_param = MPDrawParams()
    draw_param.time_begin = 0

    config.scenario.draw(rnd, draw_param)
    rnd.render()

    if config.planning.coordinate_system == "CVLN":
        rnd.ax.plot(config.planning.reference_path[:, 0],
                    config.planning.reference_path[:, 1],
                    color='g', marker='.', markersize=1, zorder=19, linewidth=2.0)
        proj_domain_border = np.asarray(config.planning.CLCS.projection_domain())
        rnd.ax.plot(proj_domain_border[:, 0], proj_domain_border[:, 1], zorder=100, color='orange')
    plt.show()


def plot_collision_checker(reach_interface: ReachableSetInterface):
    """
    Plots the collision checker used by reach_interface
    """
    rnd = MPRenderer(figsize=(20, 10))
    cc = reach_interface.collision_checker
    cc.draw(rnd)
    rnd.render()

    if reach_interface.config.planning.coordinate_system == "CVLN":
        curv_projection_domain_border = np.asarray(reach_interface.config.planning.CLCS.curvilinear_projection_domain())
        rnd.ax.plot(curv_projection_domain_border[:, 0], curv_projection_domain_border[:, 1], zorder=100,
                    color='orange')
    plt.show()

def draw_vehicle_and_three_circles(
    renderer: MPRenderer,
    reach_interface: ReachableSetInterface,
    position: np.array,
    orientation: float,
    position_is_center: bool = True
) -> None:
    """Function draws ego vehicle together with its three circle occupancy approximation"""

    vehicle_config = reach_interface.config.vehicle.ego

    rad, dist = compute_disc_radius_and_distance(
        vehicle_config.length, vehicle_config.width,
        ref_point="REAR", dist_axle_rear=vehicle_config.wb_rear_axle)

    rotation_vector = np.array([np.cos(orientation), np.sin(orientation)])

    if position_is_center:
        center_position = position
        rear_axle_pos = position - vehicle_config.wb_rear_axle * rotation_vector
    else:
        center_position = position + vehicle_config.wb_rear_axle * rotation_vector
        rear_axle_pos = position

    # draw ego rectangle
    ego_shape = Rectangle(
        length=vehicle_config.length, width=vehicle_config.width,
        center=center_position, orientation=orientation)
    renderer.draw_params.shape.facecolor = "red"
    renderer.draw_params.shape.edgecolor = "red"
    ego_shape.draw(renderer)

    # draw three circle
    draw_three_disc_occupancy(
        rear_axle_pos,
        orientation,
        rad,
        dist,
        renderer
    )


def draw_three_disc_occupancy(
        pos_ego_rear_axle: np.ndarray,
        theta_ego: float,
        disc_radius: float,
        disc_distance: float,
        renderer: MPRenderer,
) -> None:
    """
    Function draws three disc approximation of ego vehicle occupancy for a given ego vehicle state.
    :pos_ego_rear_axle: position [x,y] of the ego vehicle at the rear axle as numpy array
    :theta_ego: orientation of the ego vehicle
    :disc_radius: radius of the approximating discs
    :disc_distance: distance between centers of first and third disc
    """
    # centers of discs
    center1 = pos_ego_rear_axle
    center2 = pos_ego_rear_axle + [(disc_distance / 2) * np.cos(theta_ego),
                                   (disc_distance / 2) * np.sin(theta_ego)]
    center3 = pos_ego_rear_axle + [disc_distance * np.cos(theta_ego),
                                   disc_distance * np.sin(theta_ego)]

    # create disc circles
    disc1 = Circle(disc_radius, center1)
    disc2 = Circle(disc_radius, center2)
    disc3 = Circle(disc_radius, center3)

    # set draw params
    draw_params_circle = ShapeParams()
    draw_params_circle.facecolor = "grey"
    draw_params_circle.edgecolor = "grey"
    draw_params_circle.opacity = 0.3

    # draw discs
    disc1.draw(renderer, draw_params=draw_params_circle)
    disc2.draw(renderer, draw_params=draw_params_circle)
    disc3.draw(renderer, draw_params=draw_params_circle)