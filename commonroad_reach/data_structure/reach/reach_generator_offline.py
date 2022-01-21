from typing import List, Tuple, Dict

import numpy as np
from commonroad_dc import pycrcc

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_node import ReachNodeMultiGeneration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import reach_operation


class Cell:
    cnt_id: int = 0

    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float):
        assert x_min < x_max, "<Cell> x_min should be smaller than x_max"
        assert y_min < y_max, "<Cell> y_min should be smaller than y_max"

        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.center = np.array([(x_max + x_min) / 2, (y_max + y_min) / 2])

        self.id = Cell.cnt_id
        Cell.cnt_id += 1

    def __repr__(self):
        return f"Cell(id={self.id}, x=[{self.x_min}, {self.x_max}], y=[{self.y_min}, {self.y_max}])"


class Grid:
    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float, size_grid: float):
        assert x_min < x_max, "<Grid> x_min should be smaller than x_max"
        assert y_min < y_max, "<Grid> y_min should be smaller than y_max"

        self.list_cells = list()
        self.size_grid = size_grid

        self.x_min_cells = np.floor(x_min / size_grid) * size_grid
        self.x_max_cells = np.ceil(x_max / size_grid) * size_grid
        self.y_min_cells = np.floor(y_min / size_grid) * size_grid
        self.y_max_cells = np.ceil(y_max / size_grid) * size_grid

        self.list_x_cells = np.arange(self.x_min_cells, self.x_max_cells + 0.001, self.size_grid)
        self.list_y_cells = np.arange(self.y_min_cells, self.y_max_cells + 0.001, self.size_grid)
        self.num_columns = len(self.list_x_cells) - 1
        self.num_rows = len(self.list_y_cells) - 1

        self._create_cells()

    def _create_cells(self):
        for x_min_cell, x_max_cell in zip(self.list_x_cells[:-1], self.list_x_cells[1:]):
            for y_min_cell, y_max_cell in zip(self.list_y_cells[:-1], self.list_y_cells[1:]):
                self.list_cells.append(Cell(x_min_cell, x_max_cell, y_min_cell, y_max_cell))

    def __repr__(self):
        return f"Grid(#cols={self.num_columns}, #rows={self.num_rows}, x=[{self.x_min_cells}, {self.x_max_cells}], " \
               f"y=[{self.y_min_cells}, {self.y_max_cells}], size_grid={self.size_grid})"


class OfflineReachableSetGenerator:
    def __init__(self, config: Configuration):
        self.config = config

        assert self.config.reachable_set.mode == 1, "<OfflineReachableSetGenerator> Offline reachable set generation only supports Python backend."
        assert self.config.planning.coordinate_system == "CART", "<OfflineReachableSetGenerator> Offline reachable set generation only supports Cartesian coordinate system."

        self._initialize_zero_state_polygons()

    def _initialize_zero_state_polygons(self):
        """Initializes the zero-state polygons of the system.

        Computation of the reachable set of an LTI system requires the zero-state response
        and the zero-input response of the system.
        """
        self._polygon_zero_state_lon = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                 self.config.vehicle.ego.a_lon_min,
                                                                                 self.config.vehicle.ego.a_lon_max)

        self._polygon_zero_state_lat = reach_operation.create_zero_state_polygon(self.config.planning.dt,
                                                                                 self.config.vehicle.ego.a_lat_min,
                                                                                 self.config.vehicle.ego.a_lat_max)

    @property
    def polygon_zero_state_lon(self) -> ReachPolygon:
        return self._polygon_zero_state_lon

    @property
    def polygon_zero_state_lat(self) -> ReachPolygon:
        return self._polygon_zero_state_lat

    @property
    def initial_drivable_area(self) -> List[ReachPolygon]:
        """Drivable area at the initial time step.

        Constructed directly from the config file.
        """
        tuple_vertices = reach_operation.generate_tuple_vertices_position_rectangle_initial(self.config)

        return [ReachPolygon.from_rectangle_vertices(*tuple_vertices)]

    @property
    def initial_reachable_set(self) -> List[ReachNodeMultiGeneration]:
        """Reachable set at the initial time step.

        Vertices of the longitudinal and lateral polygons are constructed directly from the config file.
        """
        tuple_vertices_polygon_lon, tuple_vertices_polygon_lat = \
            reach_operation.generate_tuples_vertices_polygons_initial(self.config)

        polygon_lon = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lon)
        polygon_lat = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lat)

        return [ReachNodeMultiGeneration(polygon_lon, polygon_lat, self.config.planning.time_step_start)]

    def compute_drivable_area_at_time_step(self, time_step: int, reachable_set_previous: List[ReachNodeMultiGeneration]) \
            -> Tuple[List[ReachPolygon], List[ReachNodeMultiGeneration]]:
        """Computes the drivable area at the specified time step.

        Steps:
            1. Propagate each node of the reachable set from the last time step.
               This forms a list of propagated base sets.
            2. Project the base sets onto the position domain.
            3. Merge and repartition these rectangles.
        """
        if len(reachable_set_previous) < 1:
            return [], []

        list_base_sets_propagated = self._propagate_reachable_set(reachable_set_previous)
        list_rectangles_projected = reach_operation.project_base_sets_to_position_domain(list_base_sets_propagated)
        list_rectangles_repartitioned = reach_operation.create_repartitioned_rectangles(
            list_rectangles_projected, self.config.reachable_set.size_grid)
        list_rectangles_adapted = self._adapt_rectangles_to_grid(list_rectangles_repartitioned,
                                                                 self.config.reachable_set.size_grid)
        drivable_area = self._remove_rectangles_out_of_kamms_circle(time_step, list_rectangles_adapted)

        return drivable_area, list_base_sets_propagated

    def _propagate_reachable_set(self, list_nodes: List[ReachNodeMultiGeneration]) -> List[ReachNodeMultiGeneration]:
        """Propagates the nodes of the reachable set from the last time step."""
        list_base_sets_propagated = []

        for node in list_nodes:
            try:
                polygon_lon_propagated = reach_operation.propagate_polygon(node.polygon_lon,
                                                                           self._polygon_zero_state_lon,
                                                                           self.config.planning.dt,
                                                                           self.config.vehicle.ego.v_lon_min,
                                                                           self.config.vehicle.ego.v_lon_max)

                polygon_lat_propagated = reach_operation.propagate_polygon(node.polygon_lat,
                                                                           self._polygon_zero_state_lat,
                                                                           self.config.planning.dt,
                                                                           self.config.vehicle.ego.v_lat_min,
                                                                           self.config.vehicle.ego.v_lat_max)
            except (ValueError, RuntimeError):
                pass

            else:
                base_set_propagated = ReachNodeMultiGeneration(polygon_lon_propagated, polygon_lat_propagated,
                                                               node.time_step)
                base_set_propagated.source_propagation = node
                list_base_sets_propagated.append(base_set_propagated)

        return list_base_sets_propagated

    def _adapt_rectangles_to_grid(self, list_rectangles: List[ReachPolygon], size_grid: float) -> List[ReachPolygon]:
        list_rectangles_adapted = []
        tuple_extremum = reach_operation.compute_extremum_positions_of_rectangles(list_rectangles)

        grid = Grid(*tuple_extremum, size_grid)

        for rectangle in list_rectangles:
            for cell in grid.list_cells:
                if not self.is_disjoint(rectangle, cell):
                    list_rectangles_adapted.append(self.intersect_rectangle_with_cell(rectangle, cell))

        return list_rectangles_adapted

    @staticmethod
    def is_disjoint(rectangle: ReachPolygon, cell: Cell) -> bool:
        if rectangle.p_lon_max < cell.x_min or rectangle.p_lon_min > cell.x_max or \
                rectangle.p_lat_max < cell.y_min or rectangle.p_lat_min > cell.y_max:
            return True

        return False

    @staticmethod
    def intersect_rectangle_with_cell(rectangle: ReachPolygon, cell: Cell) -> ReachPolygon:
        rectangle_intersected = rectangle.clone(convexify=False)
        rectangle_intersected = rectangle_intersected.intersect_halfspace(1, 0, cell.x_max)
        rectangle_intersected = rectangle_intersected.intersect_halfspace(-1, 0, -cell.x_min)
        rectangle_intersected = rectangle_intersected.intersect_halfspace(0, 1, cell.y_max)
        rectangle_intersected = rectangle_intersected.intersect_halfspace(0, -1, -cell.y_min)

        return rectangle_intersected

    def _remove_rectangles_out_of_kamms_circle(self, time_step: float,
                                               list_rectangles_adapted: List[ReachPolygon]) -> List[ReachPolygon]:
        time_duration = self.config.planning.dt * time_step
        radius_circle = 0.5 * self.config.vehicle.ego.a_max * time_duration ** 2
        collision_circle = pycrcc.Circle(radius_circle, 0.0, 0.0)

        list_idx_rectangles_to_be_deleted = list()
        for index, rectangle in enumerate(list_rectangles_adapted):
            collision_rectangle = \
                pycrcc.RectAABB((rectangle.p_lon_max - rectangle.p_lon_min) / 2,
                                (rectangle.p_lat_max - rectangle.p_lat_min) / 2,
                                (rectangle.p_lon_max + rectangle.p_lon_min) / 2,
                                (rectangle.p_lat_max + rectangle.p_lat_min) / 2)

            if not collision_circle.collide(collision_rectangle):
                list_idx_rectangles_to_be_deleted.append(index)

        return [rectangle for index, rectangle in enumerate(list_rectangles_adapted)
                if index not in list_idx_rectangles_to_be_deleted]

    @staticmethod
    def compute_reachable_set_at_time_step(time_step: int, base_set_propagated, drivable_area) \
            -> List[ReachNodeMultiGeneration]:
        """Computes the reachable set at the specified time step.

        Steps:
            1. create a list of new base sets cut down with rectangles of
               the drivable area.
            2. create the list of nodes for the final reachable set.
        """
        if not drivable_area:
            return []

        # Step 1
        list_base_sets_adapted = reach_operation.adapt_base_sets_to_drivable_area(drivable_area,
                                                                                  base_set_propagated,
                                                                                  has_multi_generation=True)
        # Step 2
        reachable_set_time_step_current = reach_operation.create_nodes_of_reachable_set(time_step,
                                                                                        list_base_sets_adapted)

        return reachable_set_time_step_current

    def determine_grandparent_relationship(self, dict_time_to_reachable_set: Dict[int, List[ReachNodeMultiGeneration]]):
        for time_step in list(dict_time_to_reachable_set.keys())[2:]:
            list_nodes = dict_time_to_reachable_set[time_step]
            list_nodes_grand_parent = dict_time_to_reachable_set[time_step - 2]
            list_nodes_grand_parent_propagated = list_nodes_grand_parent

            for _ in range(2):
                list_nodes_grand_parent_propagated = self._propagate_reachable_set(list_nodes_grand_parent_propagated)

            for node in list_nodes:
                rectangle_node = node.position_rectangle

                for idx_grandparent, node_grand_parent_propagated in enumerate(list_nodes_grand_parent_propagated):
                    rectangle_node_grand_parent_propagated = node_grand_parent_propagated.position_rectangle
                    node_grand_parent = list_nodes_grand_parent[idx_grandparent]

                    if rectangle_node.intersects(rectangle_node_grand_parent_propagated):
                        node.add_grandparent_node(node_grand_parent)
                        node_grand_parent.add_grandchild_node(node)
