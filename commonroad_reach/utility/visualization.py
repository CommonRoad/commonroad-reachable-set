import logging
from typing import Tuple, Union, List, Dict
import warnings
from copy import deepcopy

logger = logging.getLogger(__name__)
logging.getLogger('PIL').setLevel(logging.WARNING)
logging.getLogger('matplotlib.font_manager').setLevel(logging.WARNING)

from pathlib import Path
import imageio
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, PolyCollection

# commonroad
from commonroad.geometry.shape import Polygon, Rectangle
from commonroad.visualization.mp_renderer import MPRenderer

# commonroad_reach
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import coordinate_system as util_coordinate_system
from commonroad_reach.utility.general import create_lanelet_network_from_ids
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon



def plot_scenario_with_reachable_sets(reach_interface: ReachableSetInterface, time_step_end: int = 0,
                                      plot_limits: Union[List, None] = None, path_output: str = None,
                                      as_svg: bool = False):
    config = reach_interface.config
    scenario = config.scenario
    planning_problem = config.planning_problem
    ref_path = config.planning.reference_path

    backend = "CPP" if config.reachable_set.mode in [3, 5] else "PYTHON"
    time_step_end = time_step_end or reach_interface.time_step_end
    plot_limits = plot_limits or compute_plot_limits_from_reachable_sets(reach_interface, backend)

    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": edge_color}}}

    message = "* Plotting reachable sets..."
    print(message)
    logger.info(message)

    renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))
    for time_step in range(time_step_end + 1):
        # clear plot
        plt.cla()
        list_nodes = reach_interface.reachable_set_at_time_step(time_step)

        scenario.draw(renderer, draw_params={"dynamic_obstacle": {"draw_icon": config.debug.draw_icons},
                                             "trajectory": {"draw_trajectory": True},
                                             "time_begin": time_step})
        if config.debug.draw_planning_problem:
            planning_problem.draw(renderer, draw_params={'planning_problem': {'initial_state': {'state': {
                'draw_arrow': False, "radius": 0.5}}}})

        draw_reachable_sets(list_nodes, config, renderer, draw_params, backend)

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
            renderer.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19, linewidth=1.0)

        if config.debug.save_plots:
            save_fig(as_svg, path_output, time_step)

    if config.debug.save_plots and not as_svg:
        make_gif(path_output, "reachset_", time_step_end, str(scenario.scenario_id))

    message = "\tReachable sets plotted."
    print(message)
    logger.info(message)


def compute_plot_limits_from_reachable_sets(reach_interface: ReachableSetInterface, backend: str, margin: int = 20):
    config = reach_interface.config
    x_min = y_min = np.infty
    x_max = y_max = -np.infty

    if config.planning.coordinate_system == "CART":
        for time_step in range(reach_interface.time_step_start, reach_interface.time_step_end):
            if backend == "PYTHON":
                for rectangle in reach_interface.drivable_area_at_time_step(time_step):
                    bounds = rectangle.bounds
                    x_min = min(x_min, bounds[0])
                    y_min = min(y_min, bounds[1])
                    x_max = max(x_max, bounds[2])
                    y_max = max(y_max, bounds[3])

            else:
                for rectangle in reach_interface.drivable_area_at_time_step(time_step):
                    x_min = min(x_min, rectangle.p_lon_min())
                    y_min = min(y_min, rectangle.p_lat_min())
                    x_max = max(x_max, rectangle.p_lon_max())
                    y_max = max(y_max, rectangle.p_lat_max())

    elif config.planning.coordinate_system == "CVLN":
        for time_step in range(reach_interface.time_step_start, reach_interface.time_step_end):
            for rectangle_cvln in reach_interface.drivable_area_at_time_step(time_step):
                list_rectangles_cart = util_coordinate_system.convert_to_cartesian_polygons(rectangle_cvln,
                                                                                            config.planning.CLCS, False)
                for rectangle_cart in list_rectangles_cart:
                    bounds = rectangle_cart.bounds
                    x_min = min(x_min, bounds[0])
                    y_min = min(y_min, bounds[1])
                    x_max = max(x_max, bounds[2])
                    y_max = max(y_max, bounds[3])

    return [x_min - margin, x_max + margin, y_min - margin, y_max + margin]


def draw_reachable_sets(list_nodes, config, renderer, draw_params, backend):
    if config.planning.coordinate_system == "CART":
        for node in list_nodes:
            vertices = node.position_rectangle.vertices if backend == "PYTHON" else node.position_rectangle().vertices()
            Polygon(vertices=np.array(vertices)).draw(renderer, draw_params=draw_params)

    elif config.planning.coordinate_system == "CVLN":
        for node in list_nodes:
            position_rectangle = node.position_rectangle if backend == "PYTHON" else node.position_rectangle()
            list_polygons_cart = util_coordinate_system.convert_to_cartesian_polygons(position_rectangle,
                                                                                      config.planning.CLCS, True)
            for polygon in list_polygons_cart:
                Polygon(vertices=np.array(polygon.vertices)).draw(renderer, draw_params=draw_params)


def save_fig(as_svg, path_output, time_step):
    if as_svg:
        # save as svg
        plt.savefig(f'{path_output}{"reachset"}_{time_step:05d}.svg', format="svg", bbox_inches="tight",
                    transparent=False)
    else:
        # save as png
        plt.savefig(f'{path_output}{"reachset"}_{time_step:05d}.png', format="png", bbox_inches="tight",
                    transparent=False)


def make_gif(path: str, prefix: str, number_of_figures: int, file_save_name="animation", duration=0.1):
    images = []
    filenames = []

    for i in range(number_of_figures):
        im_path = path + prefix + "{:05d}.png".format(i)
        filenames.append(im_path)

    for filename in filenames:
        images.append(imageio.imread(filename))

    imageio.mimsave(path + "/../" + file_save_name + ".gif", images, duration=duration)


def plot_scenario_with_driving_corridor(driving_corridor, dc_id: int, reach_interface: ReachableSetInterface,
                                        time_step_end: Union[int, None] = None, animation: bool = False, as_svg=False):
    """
    2D-Visualization of a given driving corridor and scenario
    :param driving_corridor: Driving corridor to visualize
    :param dc_id: Id of driving corridor (idx in DC list)
    :param reach_interface: ReachableSetInterface object
    :param time_step_end: end time step (if None: the entire driving corridor is plotted in a stacked visualization)
    :param animation: make gif (works only if time_step_end is given)
    :param as_svg: save figures as svg for nice paper plots
    """
    # set ups
    config = reach_interface.config
    scenario = config.scenario
    planning_problem = config.planning_problem
    ref_path = config.planning.reference_path
    backend = "CPP" if config.reachable_set.mode in [3, 5] else "PYTHON"

    # set color
    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": edge_color}}}

    message = ("* Plotting driving corridor no. %s ..." % dc_id)
    print(message)
    logger.info(message)

    # create output directory
    path_output = config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    # set plot limits & create renderer
    plot_limits = config.debug.plot_limits or compute_plot_limits_from_reachable_sets(reach_interface, backend)
    renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))

    if time_step_end is None:
        # draw only complete driving corridor over all time steps (stacked)
        plt.cla()
        scenario.draw(renderer, draw_params={"time_begin": 0, "lanelet": {"show_label": True}})
        # draw planning problem
        if config.debug.draw_planning_problem:
            planning_problem.draw(renderer, draw_params={'planning_problem': {'initial_state': {'state': {
                'draw_arrow': False, "radius": 0.5}}}})
        # all reach set nodes in driving corridor
        list_nodes = [item for sublist in list(driving_corridor.values()) for item in sublist]
        draw_reachable_sets(list_nodes, config, renderer, draw_params, backend)

        # plot
        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        plt.margins(0, 0)
        renderer.render()

        # draw reference path
        if config.debug.draw_ref_path and ref_path is not None:
            renderer.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19,
                             linewidth=3.0)

        if config.debug.save_plots:
            plt.savefig(f'{path_output}{"lon_driving_corridor"}_{dc_id}_complete.png',
                        format="png", bbox_inches="tight", transparent=False)
    else:
        # make separate plot of driving corridor for each time step + draw stacked corridor + create gif (optional)
        assert time_step_end in range(reach_interface.time_step_end + 1), "specified end time step for visualization is" \
                                                                          "too high"

        # create separate output folder
        path_output_lon_dc = path_output + ('lon_driving_corridor_%s/' % dc_id)
        Path(path_output_lon_dc).mkdir(parents=True, exist_ok=True)

        for time_step in range(time_step_end + 1):
            # plot driving corridor and scenario at the specified time step
            plt.cla()
            scenario.draw(renderer, draw_params={"dynamic_obstacle": {"draw_icon": True}, "time_begin": time_step})
            # draw planning problem
            if config.debug.draw_planning_problem:
                planning_problem.draw(renderer, draw_params={'planning_problem': {'initial_state': {'state': {
                    'draw_arrow': False, "radius": 0.5}}}})
            # reach set nodes in driving corridor at specified time step
            list_nodes = driving_corridor[time_step]
            draw_reachable_sets(list_nodes, config, renderer, draw_params, backend)

            # plot
            plt.rc("axes", axisbelow=True)
            ax = plt.gca()
            ax.set_aspect("equal")
            plt.margins(0, 0)
            renderer.render()

            # draw reference path
            if config.debug.draw_ref_path and ref_path is not None:
                renderer.ax.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=1, zorder=19,
                                 linewidth=3.0)

            if config.debug.save_plots:
                save_format = "svg" if as_svg else "png"
                plt.savefig(
                    f'{path_output_lon_dc}{"lon_driving_corridor"}_{time_step:05d}.{save_format}',
                    format=save_format, bbox_inches="tight", transparent=False)

        if config.debug.save_plots and animation:
            if as_svg:
                warnings.warn("GIF creation not possible if as_svg option is set to True")
            make_gif(path_output_lon_dc, "lon_driving_corridor_", time_step_end, ("lon_driving_corridor_%s" % dc_id))

    message = ("\tDriving corridor %s plotted." % dc_id)
    print(message)
    logger.info(message)


def plot_all_driving_corridors(list_driving_corridors: List, reach_interface: ReachableSetInterface):
    """
    Visualizes all driving corridors in the given list of driving corridors (only for visualizing the complete driving
    corridor as stacked visualization)
    """
    dc_counter = 0
    for dc in list_driving_corridors:
        plot_scenario_with_driving_corridor(dc, dc_counter, reach_interface, time_step_end=None, animation=False)
        dc_counter += 1


def draw_driving_corridor_3d(driving_corridor: Dict, dc_id, reach_interface: ReachableSetInterface,
                             lanelet_ids: List[int] = None, list_obstacles: List = None, as_svg=False):
    """
    Draws a driving corridor with 3D projection
    """
    message = "* Plotting 3D driving corridor ..."
    print(message)
    logger.info(message)

    # get settings from config
    config = reach_interface.config
    lanelet_network = create_lanelet_network_from_ids(config.scenario.lanelet_network, lanelet_ids) if lanelet_ids \
        else config.scenario.lanelet_network
    backend = "CPP" if config.reachable_set.mode in [3, 5] else "PYTHON"

    # temporal length of driving corridor
    time_step_end = len(driving_corridor) - 1

    # setup figure
    fig = plt.figure(figsize=(20, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.computed_zorder = False
    palette = sns.color_palette("GnBu_d", 3)
    # set plot limits
    plot_limits = config.debug.plot_limits or compute_plot_limits_from_reachable_sets(reach_interface, backend)

    # create output directory
    path_output = config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    # settings for 3d view
    interval = 0.08
    height = 0.02

    # 3D rendering of lanelet network
    _render_lanelet_network_3d(lanelet_network, ax)

    for time_step in range(time_step_end + 1):
        # determine tuple (z_min, z_max) for polyhedron
        z_tuple = (time_step * interval, time_step * interval + height)

        # 3D rendering of obstacles
        if list_obstacles:
            for obs in list_obstacles:
                occ_rectangle = obs.occupancy_at_time(time_step).shape
                _render_obstacle_3d(occ_rectangle, ax, z_tuple)

        # 3D rendering of reachable sets
        list_reach_nodes = driving_corridor[time_step]
        _render_reachable_sets_3d(list_reach_nodes, ax, z_tuple, config, palette)

    # axis settings
    ax.set_xlim(plot_limits[0:2])
    ax.set_ylim(plot_limits[2:4])
    ax.set_zlim([0, 5])
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
    cosys = config.planning.coordinate_system
    backend = "CPP" if config.reachable_set.mode in [3, 5] else "PYTHON"

    # set colors
    face_color = palette[0]
    edge_color = (palette[0][0] * 0.30, palette[0][1] * 0.30, palette[0][2] * 0.30)

    if cosys == "CART":
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
    elif cosys == "CVLN":
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
        z = np.zeros(x.shape)

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


# TODO: can be merged with _compute_vertices_of_polyhedron
def _render_obstacle_3d(occupancy_rect: Rectangle, ax,  z_tuple: Tuple):
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
    pc.set_alpha(0.7)
    pc.set_zorder(20)
    ax.add_collection3d(pc)


