import logging
from typing import Tuple

logger = logging.getLogger(__name__)
logging.getLogger('PIL').setLevel(logging.WARNING)
logging.getLogger('matplotlib.font_manager').setLevel(logging.WARNING)

from pathlib import Path
import imageio
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

from commonroad.geometry.shape import Polygon
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import coordinate_system as util_coordinate_system


def plot_scenario_with_reachable_sets(reach_interface: ReachableSetInterface, time_step_end: int = 0,
                                      plot_limits: Tuple = None, path_output: str = None, as_svg: bool = False):
    config = reach_interface.config
    scenario = config.scenario
    backend = "CPP" if config.reachable_set.mode == 3 else "PYTHON"
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

        scenario.draw(renderer, draw_params={"time_begin": time_step})
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


def plot_scenario_with_driving_corridors(driving_corridors,
                                         reach_interface: ReachableSetInterface, time_step: int = None):
    """Visualizes scenario and driving corridors. For each driving corridor in the list, a separate plot is created.

    If no time step is passed (time_step=None), the complete driving corridor is shown and the scenario at first time
    step is plotted. If a time step is passed, the scenario and the driving corridor at the given time step is shown.
    """
    config = reach_interface.config
    scenario = config.scenario
    backend = "CPP" if config.reachable_set.mode == 3 else "PYTHON"

    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": edge_color}}}

    message = "* Plotting driving corridors..."
    print(message)
    logger.info(message)

    # loop over driving corridors in list and count
    dc_counter = 1
    for dc in driving_corridors:
        # create output directory
        path_output = config.general.path_output
        Path(path_output).mkdir(parents=True, exist_ok=True)

        # set plot limits
        plot_limits = compute_plot_limits_from_reachable_sets(reach_interface, backend)
        # create renderer object
        renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))

        if time_step is None:
            # plot complete driving corridor and scenario at first time step
            plt.cla()
            scenario.draw(renderer, draw_params={"time_begin": 0})
            # all reach set nodes in driving corridor
            list_nodes = [item for sublist in list(dc.values()) for item in sublist]
            draw_reachable_sets(list_nodes, config, renderer, draw_params, backend)

        else:
            # plot driving corridor and scenario at the specified time step
            plt.cla()
            scenario.draw(renderer, draw_params={"time_begin": 0})
            # reach set nodes in driving corridor at specified time step
            list_nodes = dc[time_step]
            draw_reachable_sets(list_nodes, config, renderer, draw_params, backend)

        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        plt.margins(0, 0)
        renderer.render()

        if config.debug.save_plots:
            if time_step is None:
                plt.savefig(f'{path_output}{"lon_driving_corridor"}_{dc_counter}_complete.png',
                            format="png", bbox_inches="tight", transparent=False)
            else:
                plt.savefig(
                    f'{path_output}/{"lon_driving_corridor"}_{dc_counter}/{"lon_driving_corridor"}_{time_step:05d}.png',
                    format="png", bbox_inches="tight", transparent=False)
        # next driving corridor
        dc_counter += 1

    message = "\tDriving corridors plotted."
    print(message)
    logger.info(message)
