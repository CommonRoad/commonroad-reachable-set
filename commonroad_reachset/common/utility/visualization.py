import os
import random
from pathlib import Path

import imageio
import matplotlib.pyplot as plt
import numpy as np
# import pycrreachset as reach
import seaborn as sns
from commonroad.geometry.shape import Polygon
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib.animation import FuncAnimation

from commonroad_reachset.common.data_structure.configuration import Configuration
from commonroad_reachset.common.utility import coordinate_system as util_coordinate_system


def draw_scenario_with_reachable_sets(config, reach_interface, time_step_end: int = 0, plot_limits=None, path_output=None,
                                      as_svg=False, plot_refined: bool = False, plot_spot: bool = False):
    scenario = config.scenario

    # set output path
    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    # set color
    palette = sns.color_palette("GnBu_d", 3)
    color_edge = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": color_edge}}}

    # set time steps
    if not time_step_end:
        time_step_end = reach_interface.time_step_end

    # set plot limits
    if not plot_limits:
        plot_limits = compute_plot_limits_from_reachable_sets(config, reach_interface)

    print("Plotting Reachable sets...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))
    for time_step in range(time_step_end):
        # clear plot
        plt.cla()

        # plot scenario at current time step
        scenario.draw(renderer, draw_params={"time_begin": time_step})

        if not plot_refined:
            list_nodes = reach_interface.reachable_set_at_time_step(time_step)
        else:
            list_nodes = reach_interface.refined_reachable_set_at_time_step(time_step)

        # draw nodes of reachable sets at current time step
        if config.planning.coordinate_system == "CART":
            for node in list_nodes:
                Polygon(vertices=np.array(node.position_rectangle.vertices)).draw(renderer, draw_params=draw_params)

        elif config.planning.coordinate_system == "CVLN":
            for node in list_nodes:
                list_polygons_CART = util_coordinate_system.convert_to_cartesian_polygons(
                    node.position_rectangle, config.planning.CLCS, True)

                if list_polygons_CART:
                    for polygon in list_polygons_CART:
                        Polygon(vertices=np.array(polygon.vertices)).draw(renderer, draw_params=draw_params)

        if plot_spot:
            # plot spot prediction for dynamic obstacles
            for obs in reach_interface.reachability_analysis.semantic_model.scenario_with_spot.dynamic_obstacles:
                obs.occupancy_at_time(time_step).draw(renderer, draw_params={'time_begin': time_step})

        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        ax.set_xlabel(f"$s$ [m]", fontsize=28)
        ax.set_ylabel("$d$ [m]", fontsize=28)
        renderer.render()
        plt.margins(0, 0)

        if as_svg:
            plt.savefig(f'{path_output}{"reachset"}_{time_step:05d}.svg', format="svg", bbox_inches="tight",
                        transparent=False)
        else:
            plt.savefig(f'{path_output}{"reachset"}_{time_step:05d}.png', format="png", bbox_inches="tight",
                        transparent=False)

    if not as_svg:
        make_gif(path_output, "reachset_", time_step_end)

    print("\tReachable sets plotted.")


def draw_scenario_with_reach_cpp(config: Configuration,
                                 reach_interface: reach.ReachableSetInterface,
                                 time_step_end: int = 0,
                                 plot_limits=None,
                                 path_output=None,
                                 fig_size=(15, 10),
                                 save_fig=False,
                                 save_gif=True,
                                 plot_refined: bool = False):
    # set output path
    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    # set time steps
    if not time_step_end:
        time_step_end = len(reach_interface.map_time_to_reachable_set().keys())

    # set plot limits
    if not plot_limits:
        plot_limits = compute_plot_limits_from_reachable_sets_cpp(config, reach_interface)

    # set color
    palette = sns.color_palette("GnBu_d", 3)
    color_edge = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": color_edge}}}

    if plot_refined:
        print("Plotting refined reachable sets.")

    if save_fig:
        print("Creating figures...")
        create_figures(config, reach_interface, time_step_end, plot_limits, path_output, draw_params, plot_refined)
        print("\tFigures created.")

    if save_gif:
        print("Creating gifs...")
        create_gifs(config, reach_interface, time_step_end, draw_params, plot_limits, fig_size, path_output,
                    plot_refined)
        print("\tGifs created.")


def draw_scenario_with_maneuver(config, extractor: ManeuverExtractor, time_step_end: int = 0, plot_limits=None,
                                path_output=None,
                                as_svg=False, plot_refined: bool = False, plot_spot: bool = False):
    print("Plotting Maneuver...")

    scenario = config.scenario
    if len(extractor.list_maneuvers_compliant) == 0:
        print("No compliant maneuver.")
        return None

    else:
        index_maneuver = random.randint(0, len(extractor.list_maneuvers_compliant) - 1)
        print(f"\tRandomly chose maneuver with index {index_maneuver}")
        maneuver = extractor.list_maneuvers_compliant[index_maneuver]

    # set output path
    path_output = path_output or config.general.path_output
    Path(path_output).mkdir(parents=True, exist_ok=True)

    # set color
    palette = sns.color_palette("GnBu_d", 3)
    color_edge = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": color_edge}}}

    # set time steps
    if not time_step_end:
        time_step_end = extractor.reach_interface.time_step_end

    # set plot limits
    if not plot_limits:
        plot_limits = compute_plot_limits_from_reachable_sets(config, extractor.reach_interface, 10)

    renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))
    for time_step in range(time_step_end):
        # clear plot
        plt.cla()

        # plot scenario at current time step
        scenario.draw(renderer, draw_params={"time_begin": time_step})

        # gather reach nodes from state nodes
        list_nodes = maneuver.list_nodes[time_step].list_nodes_reach

        # draw nodes of reachable sets at current time step
        if config.planning.coordinate_system == "CART":
            for node in list_nodes:
                Polygon(vertices=np.array(node.position_rectangle.vertices)).draw(renderer, draw_params=draw_params)

        elif config.planning.coordinate_system == "CVLN":
            for node in list_nodes:
                list_polygons_CART = util_coordinate_system.convert_to_cartesian_polygons(
                    node.position_rectangle, config.planning.CLCS, True)

                if list_polygons_CART:
                    for polygon in list_polygons_CART:
                        Polygon(vertices=np.array(polygon.vertices)).draw(renderer, draw_params=draw_params)

        if plot_spot:
            # plot spot prediction for dynamic obstacles
            for obs in extractor.reach_interface.semantic_model.scenario_with_spot.dynamic_obstacles:
                obs.occupancy_at_time(time_step).draw(renderer, draw_params={'time_begin': time_step})

        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        ax.set_xlabel(f"$s$ [m]", fontsize=28)
        ax.set_ylabel("$d$ [m]", fontsize=28)
        renderer.render()
        plt.margins(0, 0)

        if as_svg:
            plt.savefig(f'{path_output}{"reachset"}_{time_step:05d}.svg', format="svg", bbox_inches="tight",
                        transparent=False)
        else:
            plt.savefig(f'{path_output}{"reachset"}_{time_step:05d}.png', format="png", bbox_inches="tight",
                        transparent=False)

    if not as_svg:
        make_gif(path_output, "reachset_", time_step_end)

    print("\tManeuver plotted.")


def create_figures(config: Configuration, reach_interface: reach.ReachableSetInterface,
                   time_step_end, plot_limits, path_output, draw_params, as_svg=True, plot_refined=False):
    scenario = config.scenario
    renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))
    for time_step in range(time_step_end):
        # clear plot
        plt.cla()

        # plot scenario at current time step
        scenario.draw(renderer, draw_params={"time_begin": time_step})

        # draw rectangles of drivable area at current time step
        if not plot_refined:
            list_rectangles_drivable_area = reach_interface.reachable_set_at_time_step(time_step)
        else:
            list_rectangles_drivable_area = reach_interface.refined_reachable_set_at_time_step(time_step)

        if reach_interface.config().planning.coordinate_system == reach.CoordinateSystem.CARTESIAN:
            for rectangle in list_rectangles_drivable_area:
                Polygon(vertices=np.array(rectangle.vertices())).draw(renderer, draw_params=draw_params)

        elif reach_interface.config().planning.coordinate_system == reach.CoordinateSystem.CURVILINEAR:
            for rectangle in list_rectangles_drivable_area:
                polygon_CART = util_coordinate_system.convert_to_cartesian_polygons(rectangle,
                                                                                    config.planning.CLCS)

                if polygon_CART:
                    Polygon(vertices=np.array(polygon_CART.vertices)).draw(renderer, draw_params=draw_params)

        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        ax.set_xlabel(f"$s$ [m]", fontsize=28)
        ax.set_ylabel("$d$ [m]", fontsize=28)
        renderer.render()
        plt.margins(0, 0)

        if as_svg:
            plt.savefig(f'{path_output}{"reachset"}_{time_step:05d}.svg', format="svg", bbox_inches="tight",
                        transparent=False)
        else:
            plt.savefig(f'{path_output}{"reachset"}_{time_step:05d}.png', format="png", bbox_inches="tight",
                        transparent=False)


def create_gifs(config: Configuration, reach_interface: reach.ReachableSetInterface,
                time_step_end, draw_params, plot_limits, fig_size, path_output, plot_refined=False):
    scenario = config.scenario
    (ln,) = plt.plot([], [], animated=True)
    interval_frame = 1000 * config.planning.dt

    fig = plt.figure(figsize=fig_size, dpi=150)
    ax = fig.gca()
    ax.axis('equal')

    renderer = MPRenderer(plot_limits=plot_limits, figsize=fig_size, ax=ax)

    def animate_plot(frame):
        # clear figure
        plt.cla()

        # draw scenario
        scenario.draw(renderer, draw_params={'time_begin': frame})

        # draw rectangles of drivable area at current time step
        if not plot_refined:
            list_nodes_reachable_set = reach_interface.reachable_set_at_time_step(frame)
        else:
            list_nodes_reachable_set = reach_interface.refined_reachable_set_at_time_step(frame)

        if reach_interface.config().planning.coordinate_system == reach.CoordinateSystem.CARTESIAN:
            for node in list_nodes_reachable_set:
                Polygon(vertices=np.array(node.position_rectangle().vertices())).draw(renderer, draw_params=draw_params)

        elif reach_interface.config().planning.coordinate_system == reach.CoordinateSystem.CURVILINEAR:
            for node in list_nodes_reachable_set:
                polygon_CART = util_coordinate_system.convert_to_cartesian_polygons(
                    node.position_rectangle(), config.planning.CLCS)

                if polygon_CART:
                    Polygon(vertices=np.array(polygon_CART.vertices)).draw(renderer, draw_params=draw_params)

            # for rectangle in list_nodes_reachable_set:
            #     polygon_CART = util_coordinate_system.convert_to_cartesian_polygon(rectangle,
            #                                                                        config.planning.CLCS)
            #     if polygon_CART:
            #         Polygon(vertices=np.array(polygon_CART.vertices)).draw(renderer, draw_params=draw_params)
        else:
            raise Exception("<Visualization> Invalid coordinate system.")

        # render figure
        renderer.render()

        plt.tight_layout()
        return (ln,)

    anim = FuncAnimation(fig, animate_plot, frames=time_step_end, blit=True, interval=interval_frame)

    anim.save(os.path.join(path_output, f"../{str(scenario.scenario_id)}.gif", ), dpi=150, writer="ffmpeg")
    plt.close(fig)


def draw_scenario_with_regions(config: Configuration, semantic_model,
                               time_step_end: int = 0, plot_limits=None,
                               path_output=None, as_svg=False):
    """Plots lanelet regions"""
    scenario = config.scenario

    # set color
    num_colors = 15
    palette = sns.color_palette("rainbow", num_colors)
    color_edge = (palette[0][0] * 0.75, palette[0][1] * 0.75, palette[0][2] * 0.75)
    draw_params = {"shape": {"polygon": {"facecolor": palette[0], "edgecolor": color_edge}}}

    # set time steps
    if not time_step_end:
        time_step_end = 1

    # set plot limits
    if not plot_limits:
        margin = 10
        list_vertices = []
        for lanelet in semantic_model.lanelet_network.lanelets:
            for vertex in lanelet.center_vertices:
                list_vertices.append(vertex)

        x_min = min([x for x, y in list_vertices])
        x_max = max([x for x, y in list_vertices])
        y_min = min([y for x, y in list_vertices])
        y_max = max([y for x, y in list_vertices])

        plot_limits = [x_min - margin, x_max + margin, y_min - margin, y_max + margin]

    print("Plotting scenario with lanelet regions...")
    renderer = MPRenderer(plot_limits=plot_limits, figsize=(25, 15))
    for time_step in range(time_step_end):
        # clear plot
        plt.cla()
        # plot scenario at current time step
        scenario.lanelet_network.draw(renderer, draw_params={"time_begin": time_step})
        idx_palette = -1
        # for region in model.dict_time_step_to_list_regions_integrated[time_step]:
        for region in semantic_model.list_regions:
            idx_palette += 1
            # # plot Cartesian polygon
            # polygon_region = region.polygon_cart
            # list_vertices_cart = [(x, y) for x, y in
            #                  zip(polygon_region.exterior.coords.xy[0], polygon_region.exterior.coords.xy[1])]

            # plot Curvilinear polygon
            polygon_region = region.polygon_cvln
            list_vertices_cvln = [(s, d) for s, d in
                                  zip(polygon_region.exterior.coords.xy[0], polygon_region.exterior.coords.xy[1])]
            list_vertices_cart = []
            for (s, d) in list_vertices_cvln:
                try:
                    x, y = config.planning.CLCS.convert_to_cartesian_coords(s, d)
                    list_vertices_cart.append((x, y))

                except ValueError:
                    continue

            if list_vertices_cart:
                list_vertices_cart.append(list_vertices_cart[0])

                Polygon(vertices=np.array(list_vertices_cart)).draw(renderer, draw_params={"shape": {
                    "polygon": {"facecolor": palette[idx_palette % num_colors],
                                "edgecolor": color_edge,
                                "zorder": 10}}})

        # plot spot prediction for dynamic obstacles
        for obs in semantic_model.scenario_with_spot.dynamic_obstacles:
            obs.occupancy_at_time(time_step).draw(renderer, draw_params={'time_begin': time_step})
        # plot static obstacles
        for obs in scenario.static_obstacles:
            obs.draw(renderer)

        plt.rc("axes", axisbelow=True)
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_title(f"$t = {time_step / 10.0:.1f}$ [s]", fontsize=28)
        ax.set_xlabel(f"$s$ [m]", fontsize=28)
        ax.set_ylabel("$d$ [m]", fontsize=28)

        renderer.render()
        plt.margins(0, 0)
        plt.show()

    print("\tReachable sets plotted.")


def make_gif(path: str, prefix: str, number_of_figures: int, file_save_name="animation", duration=0.1):
    images = []
    filenames = []

    for i in range(number_of_figures):
        im_path = path + prefix + "{:05d}.png".format(i)
        filenames.append(im_path)

    for filename in filenames:
        images.append(imageio.imread(filename))

    imageio.mimsave(path + "/../" + file_save_name + ".gif", images, duration=duration)


def compute_plot_limits_from_reachable_sets(config, reach_interface, margin=20):
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
            for rectangle in reach_interface.drivable_area_at_time_step(time_step):
                [rectangle_CART] = util_coordinate_system.convert_to_cartesian_polygons(rectangle,
                                                                                        config.planning.CLCS, False)
                bounds = rectangle_CART.bounds
                x_min = min(x_min, bounds[0])
                y_min = min(y_min, bounds[1])
                x_max = max(x_max, bounds[2])
                y_max = max(y_max, bounds[3])

    return [x_min - margin, x_max + margin, y_min - margin, y_max + margin]


def compute_plot_limits_from_reachable_sets_cpp(config: Configuration,
                                                reach_interface: reach.ReachableSetInterface, margin=20):
    x_min = y_min = np.infty
    x_max = y_max = -np.infty

    if config.planning.coordinate_system == "CART":
        for list_rectangles_drivable_area in reach_interface.map_time_to_drivable_area().values():
            for rectangle in list_rectangles_drivable_area:
                bounds = rectangle.bounds
                x_min = min(x_min, bounds[0])
                y_min = min(y_min, bounds[1])
                x_max = max(x_max, bounds[2])
                y_max = max(y_max, bounds[3])

    elif config.planning.coordinate_system == "CVLN":
        for list_rectangles_drivable_area in reach_interface.map_time_to_drivable_area().values():
            for rectangle in list_rectangles_drivable_area:
                rectangle_CART = util_coordinate_system.convert_to_cartesian_polygons(rectangle,
                                                                                      config.planning.CLCS)
                if rectangle_CART:
                    bounds = rectangle_CART.bounds
                    x_min = min(x_min, bounds[0])
                    y_min = min(y_min, bounds[1])
                    x_max = max(x_max, bounds[2])
                    y_max = max(y_max, bounds[3])

    return [x_min - margin, x_max + margin, y_min - margin, y_max + margin]
