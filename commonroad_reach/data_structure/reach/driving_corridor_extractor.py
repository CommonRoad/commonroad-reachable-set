import logging
from typing import Union, List, Dict

import numpy as np
import networkx as nx
from commonroad.geometry.shape import Shape, ShapeGroup

from commonroad_reach import pycrreach
from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_node import ReachNode, ReachPolygon
from commonroad_reach.data_structure.reach.driving_corridor import DrivingCorridor, ConnectedComponent
from commonroad_reach.utility import geometry as util_geometry
from commonroad_reach.utility import logger as util_logger
from commonroad_reach.utility import reach_operation as util_reach_operation

logger = logging.getLogger("REACH_LOGGER")


class DrivingCorridorExtractor:
    """
    Class to extract driving corridors from reachable sets and drivable areas.
    """

    def __init__(self, reachable_sets: Dict[int, List[Union[pycrreach.ReachNode, ReachNode]]], config: Configuration):
        self.reachable_sets = reachable_sets
        self.config = config
        self.backend = "CPP" if config.reachable_set.mode_computation == 2 else "PYTHON"

        util_logger.print_and_log_debug(logger, "Driving corridor extractor initialized.")

    @property
    def steps(self):
        return sorted(list(self.reachable_sets.keys()))

    def extract(self, to_goal_region: bool = False, shape_terminal: Shape = None, is_cartesian_shape: bool = True,
                corridor_lon: DrivingCorridor = None, list_p_lon: List[float] = None) -> List[DrivingCorridor]:
        """
        Extracts driving corridors within the reachable sets.

        If a longitudinal DC and a list of positions are given, lateral DCs are extracted. Otherwise,
        proceed to longitudinal DC extraction. Optionally, one can specify whether the longitudinal DC should reach
        the goal region of the planning problem or a user-given terminal states represented by a shape.

        :param to_goal_region: whether a driving corridor should end in the goal region of the planning problem
        :param shape_terminal: terminal positions represented by a CR Shape object
        :param is_cartesian_shape: flag indicating whether the shape is described in Cartesian coordinate system
        :param corridor_lon: a longitudinal driving corridor
        :param list_p_lon: a list of positions in the longitudinal direction
        :return: a list of extracted driving corridors
        """
        list_shapes_terminal = self._determine_terminal_shapes(to_goal_region, shape_terminal)

        list_corridors = list()
        for shape in list_shapes_terminal:
            list_nodes_terminal = self._determine_terminal_nodes(shape, is_cartesian_shape, corridor_lon, list_p_lon)
            list_corridors += self._extract_driving_corridors(list_nodes_terminal, corridor_lon, list_p_lon)

        list_corridors.sort(key=lambda dc: dc.area, reverse=True)

        if not corridor_lon and not list_p_lon:
            util_logger.print_and_log_info(logger, f"\tNo. of longitudinal DCs: \t{len(list_corridors)}")
        else:
            util_logger.print_and_log_info(logger, f"\tNo. of lateral DCs: \t{len(list_corridors)}")

        return list_corridors

    def _determine_terminal_shapes(self, to_goal_region: bool = False, shape_terminal: Shape = None):
        """
        Determines the terminal shapes for driving corridor extraction.

        If to_goal_region is set to true, set the goal region of the planning problem as the terminal shape.
        Otherwise, set the given terminal shape as the terminal shape.

        :param to_goal_region: whether a driving corridor should end in the goal region of the planning problem
        :param shape_terminal: terminal positions represented by a CR Shape object
        """
        if to_goal_region:
            # extract all driving corridors reaching the goal region represented by a shape(group)
            shape_goal = self.config.planning_problem.goal.state_list[0].position
            if isinstance(shape_goal, ShapeGroup):
                list_shapes_terminal = [shape for shape in shape_goal.shapes]

            else:
                list_shapes_terminal = [shape_goal]

        else:
            # extract all driving corridors reaching the goal region represented by a shape(group)
            list_shapes_terminal = [shape_terminal]

        return list_shapes_terminal

    def _determine_terminal_nodes(self, shape_terminal: Shape = None, is_cartesian_shape: bool = True,
                                  corridor_lon: DrivingCorridor = None, list_p_lon: List[float] = None) \
            -> List[Union[pycrreach.ReachNode, ReachNode]]:
        """
        Determines terminal reach nodes that overlap with the given terminal shapes.

        :param shape_terminal: terminal positions represented by a CR Shape object
        :param is_cartesian_shape: flag indicating whether the shape is described in Cartesian coordinate system
        :param corridor_lon: a longitudinal driving corridor
        :param list_p_lon: a list of positions in the longitudinal direction
        """
        if not corridor_lon and not list_p_lon:
            # extract longitudinal driving corridors
            util_logger.print_and_log_info(logger, "\tLongitudinal DC.")

            list_nodes_final = list(self.reachable_sets[self.steps[-1]])
            if shape_terminal:
                list_nodes_terminal = self._determine_overlapping_nodes_longitudinal(list_nodes_final,
                                                                                     shape_terminal, is_cartesian_shape)
                if not list_nodes_terminal:
                    print("\tFinal reach nodes do not intersect with the input terminal shape.")
                    return []

            else:
                # use all base sets in last step
                list_nodes_terminal = list_nodes_final

        elif corridor_lon and list_p_lon is not None:
            # extract lateral driving corridors
            util_logger.print_and_log_info(logger, "\tLateral DC.")

            step_final = self.steps[-1]
            if len(list_p_lon) < len(self.steps):
                message = "#Elements in provided list of longitudinal positions is less than reachable set steps."
                util_logger.print_and_log_error(logger, message)
                raise ValueError(message)
            # zip steps and longitudinal positions
            list_p_lon = list_p_lon[0:step_final + 1]
            dict_step_to_p_lon = dict(zip(self.steps, list_p_lon))

            list_nodes_terminal = \
                util_reach_operation.determine_overlapping_nodes_with_lon_pos(
                    corridor_lon.reach_nodes_at_step(step_final),
                    dict_step_to_p_lon[step_final])

        else:
            message = "Please provide both longitudinal positions and a longitudinal driving corridor if you wish to " \
                      "compute a lateral driving corridor"
            util_logger.print_and_log_error(logger, message)
            raise ValueError(message)

        return list_nodes_terminal

    def _extract_driving_corridors(self, list_nodes_terminal, corridor_lon: DrivingCorridor = None,
                                   list_p_lon: List[float] = None) -> List[DrivingCorridor]:
        """
        Extracts longitudinal or lateral driving corridors.

        If no parameter is passed, longitudinal driving corridors are extracted from the reachable sets.
        If a longitudinal driving corridor and a given list of longitudinal positions are given, lateral driving
        corridors are extracted.

        :param list_nodes_terminal: a list of reach nodes from which the corridor extraction should be performed
        :param corridor_lon: a longitudinal driving corridor
        :param list_p_lon: a list of positions in the longitudinal direction
        :return: a list of extracted driving corridors
        """
        list_corridors = list()

        # zip steps and longitudinal positions
        if list_p_lon is not None:
            step_final = self.steps[-1]
            list_p_lon = list_p_lon[0:step_final + 1]
            dict_step_to_p_lon = dict(zip(self.steps, list_p_lon))
        else:
            dict_step_to_p_lon = dict()

        list_cc_terminal = util_reach_operation.determine_connected_components(list_nodes_terminal)
        for cc_terminal in list_cc_terminal:
            list_lists_ids_cc = list()

            # create a graph of connected components backward in time
            graph_cc = nx.DiGraph()
            graph_cc.add_node(cc_terminal.id, connected_component=cc_terminal)
            self._create_connected_component_graph(list_lists_ids_cc, graph_cc, cc_terminal,
                                                   corridor_lon, dict_step_to_p_lon, cc_terminal.id)

            # convert each sequence of connected components to a driving corridor
            for lists_ids_cc in list_lists_ids_cc:
                corridor = DrivingCorridor()
                for id_cc in lists_ids_cc:
                    cc = graph_cc.nodes[id_cc]["connected_component"]
                    corridor.add_connected_component(cc)

                list_corridors.append(corridor)

        if not list_corridors:
            util_logger.print_and_log_warning(logger, "\tNo driving corridor extracted!")

        list_corridors.sort(key=lambda dc: dc.area, reverse=True)

        return list_corridors

    def _determine_overlapping_nodes_longitudinal(self, list_nodes_reach, shape_terminal: Shape,
                                                  is_cartesian_shape: bool = True) \
            -> List[Union[pycrreach.ReachNode, ReachNode]]:
        """
        Determines the terminal reach nodes that overlap with the given terminal shape.

        :param list_nodes_reach: list of reach nodes at the final step
        :param shape_terminal: terminal positions represented by a CR Shape object
        :param is_cartesian_shape: flag indicating whether the shape is described in Cartesian coordinate system
        :return: list of reach nodes overlapping with terminal positions
        """
        if self.config.planning.coordinate_system == "CVLN" and not is_cartesian_shape:
            # terminal shape already in cvln coordinates
            list_x, list_y = shape_terminal.shapely_object.exterior.coords.xy
            list_vertices_shape_terminal = [vertex for vertex in zip(list_x, list_y)]

        else:
            # convert terminal shape to cvln coordinates
            CLCS = self.config.planning.CLCS
            list_vertices = shape_terminal.shapely_object.exterior.coords
            transformed_set, transformed_set_rasterized = CLCS. \
                convert_list_of_polygons_to_curvilinear_coords_and_rasterize([list_vertices], [0], 1, 4)
            list_vertices_shape_terminal = [arr.tolist() for arr in transformed_set[0][0]]

        if self.backend == 'PYTHON':
            list_terminal_set_polygons = [ReachPolygon(list_vertices_shape_terminal)]
            list_position_rectangles = [node.position_rectangle for node in list_nodes_reach]
            dict_rectangle_adjacency = util_geometry.create_adjacency_dictionary(list_terminal_set_polygons,
                                                                                 list_position_rectangles)

        else:
            list_terminal_set_polygons = [pycrreach.ReachPolygon(list_vertices_shape_terminal)]
            list_position_rectangles = [node.position_rectangle for node in list_nodes_reach]
            dict_rectangle_adjacency = pycrreach.create_adjacency_dictionary_boost(list_terminal_set_polygons,
                                                                                   list_position_rectangles)

        list_nodes_terminal = [list_nodes_reach[j] for j in dict_rectangle_adjacency[0]]

        return list_nodes_terminal

    def _create_connected_component_graph(self, list_lists_ids_cc: List[int], graph_cc: nx.Graph,
                                          cc_current: ConnectedComponent, corridor_lon: DrivingCorridor = None,
                                          dict_step_to_p_lon: Dict[int, float] = None, id_cc_terminal: int = 0):
        """
        Traverses graph of connected reachable sets backwards in time and extracts paths starting from a terminal set.

        A path within the graph corresponds to a possible driving corridor.

        :param list_lists_ids_cc: list of found driving corridors in the reachable set
        :param graph_cc: graph of possible driving corridors
        :param cc_current: currently examined connected component
        :param corridor_lon: longitudinal driving corridor (only necessary for lateral DCs)
        :param dict_step_to_p_lon: dictionary mapping step to longitudinal positions (only necessary for lateral DCs)
        :param id_cc_terminal: id of the terminal connected component
        """
        # todo: make as a config parameter?
        # terminate if enough driving corridors are found
        if len(list_lists_ids_cc) > 10:
            return

        # computation reached the initial step, extract simple paths from terminal cc to initial cc
        if cc_current.step == self.steps[0]:
            # source: id of initial cc, target: id of terminal cc
            list_lists_ids_cc.extend(nx.all_simple_paths(graph_cc, cc_current.id, id_cc_terminal))
            return

        # determine parent reach nodes for each reach node within the current connected component
        set_nodes_reach_parent = set()
        [set_nodes_reach_parent.update(reach_node.list_nodes_parent) for reach_node in cc_current.list_nodes_reach]

        list_nodes_parent_filtered = list()
        if not corridor_lon and not dict_step_to_p_lon:
            # extract longitudinal DC
            list_nodes_parent_filtered = list(set_nodes_reach_parent)

        elif corridor_lon and dict_step_to_p_lon:
            # extract lateral DC
            # consider only reach nodes that overlap with given longitudinal position
            step_parent = cc_current.step - 1
            list_nodes_parent_filtered = util_reach_operation.determine_overlapping_nodes_with_lon_pos(
                list(set_nodes_reach_parent), dict_step_to_p_lon[step_parent])

            # todo: update this message?
            if not list_nodes_parent_filtered:
                util_logger.print_and_log_warning(logger,
                                                  f'No reachboxes found at x position. #parent reach nodes: '
                                                  f'{len(set_nodes_reach_parent)}. current step {cc_current.step}')

            # filter out reach nodes that are not part of the longitudinal driving corridor
            set_nodes_reach_parent = set(list_nodes_parent_filtered)
            set_nodes_reach_parent.intersection_update(corridor_lon.reach_nodes_at_step(step_parent))
            list_nodes_parent_filtered = list(set_nodes_reach_parent)

        # determine connected components in parent reach nodes
        exclude_small_area = self.config.reachable_set.exclude_small_components_corridor and \
                             cc_current.step - self.steps[0] > 5
        cc_parent = util_reach_operation.determine_connected_components(list_nodes_parent_filtered, exclude_small_area)

        # recursion backwards in time
        for cc_next in cc_parent:
            graph_cc.add_node(cc_next.id, connected_component=cc_next)
            graph_cc.add_edge(cc_next.id, cc_current.id)
            self._create_connected_component_graph(list_lists_ids_cc, graph_cc, cc_next,
                                                   corridor_lon, dict_step_to_p_lon, id_cc_terminal)

    @staticmethod
    def _determine_area_of_driving_corridor(driving_corridor: Dict[int, List[Union[pycrreach.ReachNode, ReachNode]]]):
        """
        Function to compute the cumulative area of a driving corridor.
        """
        area = 0.0
        for time_idx, reach_set_nodes in driving_corridor.items():
            area += util_reach_operation.compute_area_of_reach_nodes(reach_set_nodes)

        return area
