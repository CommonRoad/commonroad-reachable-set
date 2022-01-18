from decimal import Decimal
from typing import List, Tuple

import numpy as np

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_node import ReachNode
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
        return f"Cell(id={self.id},x=[{self.x_min}, {self.x_max}],y=[{self.y_min}, {self.y_max}])"


class Grid:
    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float, size_grid: float):
        assert x_min < x_max, "<Grid> x_min should be smaller than x_max"
        assert y_min < y_max, "<Grid> y_min should be smaller than y_max"

        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.size_grid = size_grid

        self._create_cells()

    def _create_cells(self):
        size_grid = Decimal(self.size_grid)
        x_max = Decimal(self.x_max)
        x_min = Decimal(self.x_min)
        y_max = Decimal(self.y_max)
        y_min = Decimal(self.y_min)

        self._num_columns = int((x_max - x_min) / size_grid)
        self._num_rows = int((y_max - y_min) / size_grid)

        self._grid = np.empty([self._num_columns, self._num_rows], dtype=Cell)
        for idx_col in range(self._num_columns):
            x_min_cell = self.size_grid * idx_col
            x_max_cell = self.size_grid * (idx_col + 1)
            for idx_row in range(self._num_rows):
                y_min_cell = self.size_grid * idx_row
                y_max_cell = self.size_grid * (idx_row + 1)
                self._grid[idx_col, idx_row] = Cell(x_min_cell, x_max_cell, y_min_cell, y_max_cell)

    @property
    def list_cells(self):
        list_cells = []
        for row in self._grid:
            for cell in row:
                list_cells.append(cell)

        return list_cells

    def __repr__(self):
        return f"Grid(#cols={self._num_columns}, #rows={self._num_rows}, x=[{self.x_min}, {self.x_max}], " \
               f"y=[{self.y_min}, {self.y_max}], size_grid={self.size_grid})"


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
    def initial_reachable_set(self) -> List[ReachNode]:
        """Reachable set at the initial time step.

        Vertices of the longitudinal and lateral polygons are constructed directly from the config file.
        """
        tuple_vertices_polygon_lon, tuple_vertices_polygon_lat = \
            reach_operation.generate_tuples_vertices_polygons_initial(self.config)

        polygon_lon = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lon)
        polygon_lat = ReachPolygon.from_rectangle_vertices(*tuple_vertices_polygon_lat)

        return [ReachNode(polygon_lon, polygon_lat, self.config.planning.time_step_start)]

    def compute_drivable_area_at_time_step(self, time_step: int, reachable_set_previous: List[ReachNode]) \
            -> Tuple[List[ReachPolygon], List[ReachNode]]:
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
        drivable_area = self._adapt_rectangles_to_grid(list_rectangles_repartitioned,
                                                       self.config.reachable_set.size_grid)
        # todo: kamm's circle

        return drivable_area, list_base_sets_propagated

    def _propagate_reachable_set(self, list_nodes: List[ReachNode]) -> List[ReachNode]:
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
                base_set_propagated = ReachNode(polygon_lon_propagated, polygon_lat_propagated, node.time_step)
                base_set_propagated.source_propagation = node
                list_base_sets_propagated.append(base_set_propagated)

        return list_base_sets_propagated

    def _adapt_rectangles_to_grid(self, list_rectangles: List[ReachPolygon], size_grid: float) -> List[ReachPolygon]:
        list_rectangles_adapted = []
        tuple_extremum = reach_operation.compute_extremum_positions_of_rectangles(list_rectangles)

        grid = Grid(*tuple_extremum, size_grid)

        for cell in grid.list_cells:
            for rectangle in list_rectangles:
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
        rectangle_intersected.intersect_halfspace(1, 0, cell.x_max)
        rectangle_intersected.intersect_halfspace(-1, 0, -cell.x_min)
        rectangle_intersected.intersect_halfspace(0, 1, cell.y_max)
        rectangle_intersected.intersect_halfspace(0, -1, -cell.y_min)

        return rectangle_intersected

    @staticmethod
    def compute_reachable_set_at_time_step(time_step: int, base_set_propagated, drivable_area) -> List[ReachNode]:
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
                                                                                  base_set_propagated)
        # Step 2
        reachable_set_time_step_current = reach_operation.create_nodes_of_reachable_set(time_step,
                                                                                        list_base_sets_adapted)

        return reachable_set_time_step_current
