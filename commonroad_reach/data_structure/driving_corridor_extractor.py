import warnings
from typing import Union, List, Dict

import networkx as nx
import numpy as np
from commonroad.geometry.shape import Shape, ShapeGroup

from commonroad_reach import pycrreach
from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.driving_corridor import DrivingCorridor, ConnectedComponent
from commonroad_reach.data_structure.reach.reach_node import ReachNode, ReachPolygon
from commonroad_reach.utility import geometry as util_geometry

# scaling factor (avoid numerical errors)
DIGITS = 2


class DrivingCorridorExtractor:
    """Class to extract driving corridors from computed reachable sets and drivable areas."""

    def __init__(self, reachable_sets: Dict[int, List[Union[pycrreach.ReachNode, ReachNode]]], config: Configuration):
        self.reachable_sets = reachable_sets
        self.config = config
        self.backend = "CPP" if config.reachable_set.mode_computation == 2 else "PYTHON"

    @property
    def steps(self):
        return sorted(list(self.reachable_sets.keys()))

    def extract(self, to_goal_region: bool = False, shape_terminal: Shape = None, is_cartesian_shape: bool = True,
                corridor_lon: DrivingCorridor = None, list_p_lon: List[float] = None) -> List[DrivingCorridor]:
        """Extracts driving corridors within the reachable sets.

        If a longitudinal DC and a list of positions are given, lateral DCs are extracted. Otherwise,
        proceed to longitudinal DC extraction. Optionally, one can specify whether the longitudinal DC should reach
        the goal region of the planning problem or a user-given terminal states represented by a shape.
        """
        list_shapes_terminal = self._determine_terminal_shapes(to_goal_region, shape_terminal)

        list_corridors = list()
        for shape in list_shapes_terminal:
            list_nodes_terminal = self._determine_terminal_nodes(shape, is_cartesian_shape, corridor_lon, list_p_lon)
            list_corridors += \
                self._extract_driving_corridors(list_nodes_terminal, corridor_lon, list_p_lon)

        return list_corridors

    def _determine_terminal_shapes(self, to_goal_region: bool = False, shape_terminal: Shape = None):
        """Determines the terminal shape for driving corridor extraction.

        If to_goal_region set to true, set the goal region of the planning problem as the terminal shape.
        Otherwise, set the given terminal shape.
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

    def _extract_driving_corridors(self, list_nodes_terminal,
                                   corridor_lon: DrivingCorridor = None, dict_step_to_p_lon: Dict[int, float] = None) \
            -> List[DrivingCorridor]:
        """Extracts longitudinal or lateral driving corridors.

        If no parameter is passed, longitudinal driving corridors are extracted from the reachable sets.
        If a longitudinal driving corridor and a given list of longitudinal positions are given, lateral driving
        corridors are extracted.
        """
        list_cc_terminal = self._determine_connected_components(list_nodes_terminal)
        print("terminal cc determined.")
        list_corridors = list()
        for cc_terminal in list_cc_terminal:
            list_lists_ids_cc = list()
            graph_cc = nx.DiGraph()
            graph_cc.add_node(cc_terminal.id, connected_component=cc_terminal)
            print(f"add cc node {cc_terminal.id}")
            self._create_connected_component_graph(list_lists_ids_cc, graph_cc, cc_terminal, corridor_lon,
                                                   dict_step_to_p_lon)

            for lists_ids_cc in list_lists_ids_cc:
                corridor = DrivingCorridor()
                for id_cc in lists_ids_cc:
                    cc = graph_cc.nodes[id_cc]["connected_component"]
                    corridor.add_connected_component(cc)

                list_corridors.append(corridor)

        if not list_corridors:
            warnings.warn('\t\t\t No driving corridor found!')

        else:
            list_corridors.sort(key=lambda dc: dc.area, reverse=True)

        return list_corridors

    def _determine_terminal_nodes(self, shape_terminal: Shape = None, is_cartesian_shape: bool = True,
                                  corridor_lon: DrivingCorridor = None, list_p_lon: List[float] = None) \
            -> List[Union[pycrreach.ReachNode, ReachNode]]:
        if not corridor_lon and not list_p_lon:
            # extract longitudinal driving corridors
            print("\tExtracting longitudinal driving corridors...")
            list_nodes_final = list(self.reachable_sets[self.steps[-1]])

            if shape_terminal:
                # set reach nodes at the final step that overlap with the given terminal shape as terminal nodes
                list_nodes_terminal = self._determine_terminal_nodes_longitudinal(list_nodes_final,
                                                                                  shape_terminal, is_cartesian_shape)
                if not list_nodes_terminal:
                    print("\tFinal reach nodes do not intersect with the input terminal shape.")
                    return []

            else:
                # use all base sets in last time step
                list_nodes_terminal = list_nodes_final

        elif list_p_lon and corridor_lon:
            # extract lateral driving corridors
            print("\tExtracting lateral driving corridors...")
            assert (len(list_p_lon) == len(self.steps)), f"Position list ({len(list_p_lon)}) should have the same" \
                                                         f"length as the reachable sets ({len(self.steps)})!"

            step_final = self.steps[-1]
            # determine reachable sets which contain the longitudinal position in last time step
            dict_step_to_p_lon = dict(zip(self.steps, list_p_lon))
            list_nodes_terminal = self._determine_terminal_nodes_lateral(corridor_lon.reach_nodes_at_step(step_final),
                                                                         dict_step_to_p_lon[step_final])

        else:
            err_msg = "Please provide both longitudinal positions and a longitudinal driving corridor if you wish to " \
                      "compute a lateral driving corridor"
            raise ValueError(err_msg)

        return list_nodes_terminal

    def _determine_terminal_nodes_longitudinal(self, list_nodes_reach,
                                               shape_terminal: Shape, is_cartesian_shape: bool = True) \
            -> List[Union[pycrreach.ReachNode, ReachNode]]:
        """Determines the terminal reach nodes that overlap with the given terminal shape.

        :param list_nodes_reach list of reach nodes at the final time step
        :param shape_terminal terminal positions represented by a CR Shape object
        :param is_cartesian_shape flag indicating whether the shape is described in Cartesian coordinate system
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
            list_position_rectangles = [node.position_rectangle() for node in list_nodes_reach]
            dict_rectangle_adjacency = pycrreach.create_adjacency_dictionary_boost(list_terminal_set_polygons,
                                                                                   list_position_rectangles)

        list_nodes_terminal = [list_nodes_reach[j] for j in dict_rectangle_adjacency[0]]

        return list_nodes_terminal

    @staticmethod
    def _determine_terminal_nodes_lateral(list_nodes_reach: List[Union[pycrreach.ReachNode, ReachNode]],
                                          lon_pos: float):
        """
        Checks which drivable areas of the given reachable sets contain a given longitudinal position and returns the
        corresponding reachable sets
        :param list_nodes_reach: List of reachable set nodes
        :param lon_pos: given longitudinal positions
        :return reach_set_nodes_overlap: Set containing the reachable set nodes which overlap with longitudinal position
        """
        reach_set_nodes_overlap = set()
        for reach_node in list_nodes_reach:
            if np.greater_equal(round(lon_pos * 10.0 ** DIGITS),
                                np.floor(reach_node.p_lon_min() * 10.0 ** DIGITS)) and \
                    np.greater_equal(np.ceil(reach_node.p_lon_max() * 10.0 ** DIGITS),
                                     round(lon_pos * 10.0 ** DIGITS)):
                reach_set_nodes_overlap.add(reach_node)
        return reach_set_nodes_overlap

    def _determine_connected_components(self, list_nodes_reach, exclude_small_area: bool = False) \
            -> List[ConnectedComponent]:

        """Determines and returns the connected reachable sets in positions domain.

        Connected components are sorted according to a heuristic (area of connected reachable sets)
        :param list_nodes_reach: list of reach nodes
        :param exclude_small_area: excludes connected components with an area smaller than the threshold
        :return: list of connected reachable sets
        """
        if self.backend == "CPP":
            print("using cpp")
            overlap = pycrreach.connected_reachset_boost(list_nodes_reach, DIGITS)

        else:
            print("using python")
            overlap = util_geometry.connected_reachset_py(list_nodes_reach, DIGITS)
        print("done")
        # adjacency list: list with tuples, e.g., (0, 1) representing that node 0 and node 1 are connected
        adjacency = []
        for v in overlap.values():
            adjacency += v

        list_connected_component = list()
        # create graph with nodes = reach nodes and edges = adjacency status
        graph = nx.Graph()
        graph.add_nodes_from(list(range(len(list_nodes_reach))))
        graph.add_edges_from(adjacency)

        for set_indices_nodes_reach_connected in nx.connected_components(graph):
            list_nodes_reach_in_cc = [list_nodes_reach[idx] for idx in set_indices_nodes_reach_connected]
            connected_component = ConnectedComponent(list_nodes_reach_in_cc)

            # todo: add threshold to config?
            if exclude_small_area and len(set_indices_nodes_reach_connected) <= 2 and connected_component.area < 0.05:
                pass

            list_connected_component.append(connected_component)

        # sort connected components based on their areas
        list_connected_component.sort(key=lambda cc: cc.area, reverse=True)

        return list_connected_component

    def _create_connected_component_graph(self, list_lists_ids_cc: List[int], graph_cc: nx.Graph,
                                          cc_current: ConnectedComponent,
                                          corridor_lon: DrivingCorridor = None,
                                          list_p_lon: Dict[int, float] = None):
        """Traverses graph of connected reachable sets backwards in time and extracts paths starting from a terminal set.

        A path within the graph corresponds to a possible driving corridor
        :param list_lists_ids_cc: list of found driving corridors in the reachable set
        :param graph_cc: graph of possible driving corridors
        :param list_p_lon: longitudinal positions of longitudinal trajectory (only necessary for lateral driving corridors)
        :param corridor_lon: longitudinal driving corridor (only necessary for lateral driving corridors)
        """
        # terminate if enough driving corridors are found
        # todo: make as config
        print(f"cc current: {cc_current.id}")
        if len(list_lists_ids_cc) > 1:
            return

        # computation reached the initial time step, extract simple paths from terminal cc to initial cc
        if cc_current.step == self.steps[0]:
            # nx simple paths: graph, source node id, target node id
            list_lists_ids_cc.extend(nx.all_simple_paths(graph_cc, cc_current.id, 0))
            return

        list_nodes_reach_parent = list()
        # determine parent reach sets for each reach set within connected components
        print(f"extending parent nodes")
        if self.backend == "CPP":
            [list_nodes_reach_parent.extend(reach_node.vec_nodes_parent())
             for reach_node in cc_current.list_nodes_reach]

        else:
            [list_nodes_reach_parent.extend(reach_node.list_nodes_parent) for reach_node in cc_current.list_nodes_reach]

        print(f"parent nodes extended")
        if not corridor_lon and not list_p_lon:
            # extract longitudinal DC
            list_nodes_parent_filtered = list_nodes_reach_parent

        elif corridor_lon and list_p_lon:
            # extract lateral DC
            # for lateral driving corridor: further consider only sets that overlap with given longitudinal position
            list_nodes_parent_filtered = self._determine_terminal_nodes_lateral(list_nodes_reach_parent,
                                                                                list_p_lon[cc_current.step - 1])
            if not list_nodes_parent_filtered:
                warnings.warn(f'No reachboxes found at x position. '
                              f'#parent reach nodes: {len(list_nodes_reach_parent)}. current step {cc_current.step}')

            # filter out reach set nodes that are not part of the longitudinal driving corridor
            list_nodes_parent_filtered.intersection_update(corridor_lon.reach_nodes_at_step(cc_current.step - 1))

        else:
            err_msg = "You need to provide both longitudinal positions and a longitudinal driving corridor to " \
                      "compute a lateral driving corridor"
            raise ValueError(err_msg)

        # determine connected components in parent reach nodes
        exclude_small_area = cc_current.step > 5
        print(f"determining connected component...")
        cc_parent = self._determine_connected_components(list_nodes_parent_filtered, exclude_small_area)

        # recursion backwards in time
        for cc_next in cc_parent:
            graph_cc.add_node(cc_next.id, connected_component=cc_next)
            graph_cc.add_edge(cc_next.id, cc_current.id)
            print(f"graph add cc node {cc_next.id}")
            self._create_connected_component_graph(list_lists_ids_cc, graph_cc, cc_next, corridor_lon, list_p_lon)

    @staticmethod
    def _determine_area_of_driving_corridor(driving_corridor: Dict[int, List[Union[pycrreach.ReachNode, ReachNode]]]):
        """
        Function to compute the cumulative area of a driving corridor, i.e.,
        :param driving_corridor:
        :return: area
        """
        area = 0.0
        for time_idx, reach_set_nodes in driving_corridor.items():
            area += util_geometry.area_of_reachable_set(reach_set_nodes)

        return area
