from typing import Union, List

import imageio
import numpy as np
import seaborn as sns
from pathlib import Path
import matplotlib.pyplot as plt

from commonroad.geometry.shape import Polygon
from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_reachset.data_structure.reach.reach_node import ReachNode
from commonroad_reachset.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reachset.utility import coordinate_system as util_coordinate_system


def plot_scenario_with_reachable_sets(reach_interface: ReachableSetInterface, time_step_end=0, plot_limits=None,
                                      path_output=None, as_svg=False, plot_pruned=False):
    # ==== preparation
    config = reach_interface.config
    scenario = config.scenario

    # create output directory
    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    # set time steps
    time_step_end = time_step_end or reach_interface.time_step_end

    # set plot limits
    plot_limits = plot_limits or compute_plot_limits_from_reachable_sets(reach_interface)

    # set color
    palette = sns.color_palette("GnBu_d", 3)
    edge_color = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": edge_color}}}

    # ==== plotting
    print("\nPlotting reachable sets...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))
    for time_step in range(time_step_end):
        # clear plot
        plt.cla()

        # plot scenario at current time step
        scenario.draw(renderer, draw_params={"time_begin": time_step})

        if not plot_pruned:
            list_nodes = reach_interface.reachable_set_at_time_step(time_step)

        else:
            list_nodes = reach_interface.pruned_reachable_set_at_time_step(time_step)

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

        if config.debug.save_plots:
            save_fig(as_svg, path_output, time_step)

    if config.debug.save_plots and not as_svg:
        make_gif(path_output, "reachset_", time_step_end, str(scenario.scenario_id))

    print("\tReachable sets plotted.")


def compute_plot_limits_from_reachable_sets(reach_interface: ReachableSetInterface, margin=20):
    config = reach_interface.config
    x_min = y_min = np.infty
    x_max = y_max = -np.infty

    if config.planning.coordinate_system == "CART":
        for time_step in range(reach_interface.time_step_start, reach_interface.time_step_end):
            for rectangle in reach_interface.drivable_area_at_time_step(time_step):
                bounds = rectangle.bounds
                x_min = min(x_min, bounds[0])
                y_min = min(y_min, bounds[1])
                x_max = max(x_max, bounds[2])
                y_max = max(y_max, bounds[3])

    elif config.planning.coordinate_system == "CVLN":
        for time_step in range(reach_interface.time_step_start, reach_interface.time_step_end):
            for rectangle_CVLN in reach_interface.drivable_area_at_time_step(time_step):
                list_rectangles_CART = util_coordinate_system.convert_to_cartesian_polygons(rectangle_CVLN,
                                                                                            config.planning.CLCS, False)
                for rectangle_CART in list_rectangles_CART:
                    bounds = rectangle_CART.bounds
                    x_min = min(x_min, bounds[0])
                    y_min = min(y_min, bounds[1])
                    x_max = max(x_max, bounds[2])
                    y_max = max(y_max, bounds[3])

    return [x_min - margin, x_max + margin, y_min - margin, y_max + margin]


def draw_reachable_sets(list_nodes: Union[List[ReachNode]], config, renderer, draw_params):
    """Draws reach nodes."""
    if config.planning.coordinate_system == "CART":
        for node in list_nodes:
            Polygon(vertices=np.array(node.position_rectangle.vertices)).draw(renderer, draw_params=draw_params)

    elif config.planning.coordinate_system == "CVLN":
        for node in list_nodes:
            list_polygons_CART = util_coordinate_system.convert_to_cartesian_polygons(node.position_rectangle,
                                                                                      config.planning.CLCS, True)
            if list_polygons_CART:
                for polygon in list_polygons_CART:
                    Polygon(vertices=np.array(polygon.vertices)).draw(renderer, draw_params=draw_params)


def save_fig(as_svg, path_output, time_step):
    """Saves plots to output directory."""
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

# def compute_plot_limits_from_reachable_sets_cpp(config: Configuration,
#                                                 reach_interface: reach.ReachableSetInterface, margin=20):
#     x_min = y_min = np.infty
#     x_max = y_max = -np.infty
#
#     if config.planning.coordinate_system == "CART":
#         for list_rectangles_drivable_area in reach_interface.map_time_to_drivable_area().values():
#             for rectangle in list_rectangles_drivable_area:
#                 bounds = rectangle.bounds
#                 x_min = min(x_min, bounds[0])
#                 y_min = min(y_min, bounds[1])
#                 x_max = max(x_max, bounds[2])
#                 y_max = max(y_max, bounds[3])
#
#     elif config.planning.coordinate_system == "CVLN":
#         for list_rectangles_drivable_area in reach_interface.map_time_to_drivable_area().values():
#             for rectangle in list_rectangles_drivable_area:
#                 rectangle_CART = util_coordinate_system.convert_to_cartesian_polygons(rectangle,
#                                                                                       config.planning.CLCS)
#                 if rectangle_CART:
#                     bounds = rectangle_CART.bounds
#                     x_min = min(x_min, bounds[0])
#                     y_min = min(y_min, bounds[1])
#                     x_max = max(x_max, bounds[2])
#                     y_max = max(y_max, bounds[3])
#
#     return [x_min - margin, x_max + margin, y_min - margin, y_max + margin]

# def draw_scenario_with_reach_cpp(config: Configuration,
#                                  reach_interface: reach.ReachableSetInterface,
#                                  time_step_end: int = 0,
#                                  plot_limits=None,
#                                  path_output=None,
#                                  fig_size=(15, 10),
#                                  save_fig=False,
#                                  save_gif=True,
#                                  plot_refined: bool = False):
#     # set output path
#     path_output = path_output or config.general.path_output
#     Path(path_output).mkdir(parents=True, exist_ok=True)
#
#     # set time steps
#     if not time_step_end:
#         time_step_end = len(reach_interface.map_time_to_reachable_set().keys())
#
#     # set plot limits
#     if not plot_limits:
#         plot_limits = compute_plot_limits_from_reachable_sets_cpp(config, reach_interface)
#
#     # set color
#     palette = sns.color_palette("GnBu_d", 3)
#     color_edge = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
#     draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": color_edge}}}
#
#     if plot_refined:
#         print("Plotting refined reachable sets.")
#
#     if save_fig:
#         print("Creating figures...")
#         create_figures(config, reach_interface, time_step_end, plot_limits, path_output, draw_params, plot_refined)
#         print("\tFigures created.")
#
#     if save_gif:
#         print("Creating gifs...")
#         create_gifs(config, reach_interface, time_step_end, draw_params, plot_limits, fig_size, path_output,
#                     plot_refined)
#         print("\tGifs created.")


# def create_figures(config: Configuration, reach_interface: reach.ReachableSetInterface,
#                    time_step_end, plot_limits, path_output, draw_params, as_svg=True, plot_refined=False):
#     scenario = config.scenario
#     renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))
#     for time_step in range(time_step_end):
#         # clear plot
#         plt.cla()
#
#         # plot scenario at current time step
#         scenario.draw(renderer, draw_params={"time_begin": time_step})
#
#         # draw rectangles of drivable area at current time step
#         if not plot_refined:
#             list_rectangles_drivable_area = reach_interface.reachable_set_at_time_step(time_step)
#         else:
#             list_rectangles_drivable_area = reach_interface.pruned_reachable_set_at_time_step(time_step)
#
#         if reach_interface.config().planning.coordinate_system == reach.CoordinateSystem.CARTESIAN:
#             for rectangle in list_rectangles_drivable_area:
#                 Polygon(vertices=np.array(rectangle.vertices())).draw(renderer, draw_params=draw_params)
#
#         elif reach_interface.config().planning.coordinate_system == reach.CoordinateSystem.CURVILINEAR:
#             for rectangle in list_rectangles_drivable_area:
#                 polygon_CART = util_coordinate_system.convert_to_cartesian_polygons(rectangle,
#                                                                                     config.planning.CLCS)
#
#                 if polygon_CART:
#                     Polygon(vertices=np.array(polygon_CART.vertices)).draw(renderer, draw_params=draw_params)
#
#         plt.rc("axes", axisbelow=True)
#         ax = plt.gca()
#         ax.set_aspect("equal")
#         ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
#         ax.set_xlabel(f"$s$ [m]", fontsize=28)
#         ax.set_ylabel("$d$ [m]", fontsize=28)
#         renderer.render()
#         plt.margins(0, 0)
#
#         if as_svg:
#             plt.savefig(f'{path_output}{"reachset"}_{time_step:05d}.svg', format="svg", bbox_inches="tight",
#                         transparent=False)
#         else:
#             plt.savefig(f'{path_output}{"reachset"}_{time_step:05d}.png', format="png", bbox_inches="tight",
#                         transparent=False)


# def create_gifs(config: Configuration, reach_interface: reach.ReachableSetInterface,
#                 time_step_end, draw_params, plot_limits, fig_size, path_output, plot_refined=False):
#     scenario = config.scenario
#     (ln,) = plt.plot([], [], animated=True)
#     interval_frame = 1000 * config.planning.dt
#
#     fig = plt.figure(figsize=fig_size, dpi=150)
#     ax = fig.gca()
#     ax.axis('equal')
#
#     renderer = MPRenderer(plot_limits=plot_limits, figsize=fig_size, ax=ax)
#
#     def animate_plot(frame):
#         # clear figure
#         plt.cla()
#
#         # draw scenario
#         scenario.draw(renderer, draw_params={'time_begin': frame})
#
#         # draw rectangles of drivable area at current time step
#         if not plot_refined:
#             list_nodes_reachable_set = reach_interface.reachable_set_at_time_step(frame)
#         else:
#             list_nodes_reachable_set = reach_interface.pruned_reachable_set_at_time_step(frame)
#
#         if reach_interface.config().planning.coordinate_system == reach.CoordinateSystem.CARTESIAN:
#             for node in list_nodes_reachable_set:
#                 Polygon(vertices=np.array(node.position_rectangle().vertices())).draw(renderer, draw_params=draw_params)
#
#         elif reach_interface.config().planning.coordinate_system == reach.CoordinateSystem.CURVILINEAR:
#             for node in list_nodes_reachable_set:
#                 polygon_CART = util_coordinate_system.convert_to_cartesian_polygons(
#                     node.position_rectangle(), config.planning.CLCS)
#
#                 if polygon_CART:
#                     Polygon(vertices=np.array(polygon_CART.vertices)).draw(renderer, draw_params=draw_params)
#
#             # for rectangle in list_nodes_reachable_set:
#             #     polygon_CART = util_coordinate_system.convert_to_cartesian_polygon(rectangle,
#             #                                                                        config.planning.CLCS)
#             #     if polygon_CART:
#             #         Polygon(vertices=np.array(polygon_CART.vertices)).draw(renderer, draw_params=draw_params)
#         else:
#             raise Exception("<Visualization> Invalid coordinate system.")
#
#         # render figure
#         renderer.render()
#
#         plt.tight_layout()
#         return (ln,)
#
#     anim = FuncAnimation(fig, animate_plot, frames=time_step_end, blit=True, interval=interval_frame)
#
#     anim.save(os.path.join(path_output, f"../{str(scenario.scenario_id)}.gif", ), dpi=150, writer="ffmpeg")
#     plt.close(fig)
