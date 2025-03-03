import pyximport

pyximport.install()

import logging
from typing import Tuple, Dict

import cv2 as cv
import numpy as np

import commonroad_dc.pycrcc as pycrcc
from commonroad_reach.data_structure.configuration import PlanningConfiguration
from commonroad_reach.utility.util_py_grid_online_reach import convert_cart2pixel_coordinates_c, get_vertices_from_rect

logger = logging.getLogger("REACH_LOGGER")
USE_COLLISION = True
CONVEX_SHAPE = True


class Cell:
    """
    Class representing a cell in a grid.
    """
    cnt_id: int = 0

    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float):
        assert x_min < x_max, "x_min should be smaller than x_max!"
        assert y_min < y_max, "y_min should be smaller than y_max!"

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
    """
    Cartesian grid for discretizing reachable sets in the graph-based propagation method.
    """

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
        logger.debug(f"\tCreating cartesian grid for x=[{self.x_min_cells},  {self.x_max_cells}], "
                     f"y=[{self.y_min_cells}, {self.y_max_cells}]...")

        for x_min_cell, x_max_cell in zip(self.list_x_cells[:-1], self.list_x_cells[1:]):
            for y_min_cell, y_max_cell in zip(self.list_y_cells[:-1], self.list_y_cells[1:]):
                self.list_cells.append(Cell(x_min_cell, x_max_cell, y_min_cell, y_max_cell))

        logger.debug(f"\t#Cells in grid: {len(self.list_cells)}")

    def __repr__(self):
        return f"Grid(#cols={self.num_columns}, #rows={self.num_rows}, x=[{self.x_min_cells}, {self.x_max_cells}], " \
               f"y=[{self.y_min_cells}, {self.y_max_cells}], size_grid={self.size_grid})"


class RegularGrid:
    """
    Class for computing uniformly spatial partitioned grid of obstacles.
    """

    def __init__(self, ll: Dict[int, np.ndarray], ur: Dict[int, np.ndarray], collision_checker: pycrcc.CollisionChecker,
                 dx: float, dy: float, planning_config: PlanningConfiguration, a_lon: float, a_lat: float, t_f: float,
                 grid_shapes: Dict[int, Tuple[int, int]]):
        """
        :param ll: lower-left coordinates of reachable set at every step
        :param ur: upper-right coordinates of reachable set at current step
        :param collision_checker: collision checker for discretization queries
        :param dx: grid length x
        :param dy: grid length y
        :param planning_config: configuration related to planning
        :param a_lon: acceleration in the longitudinal direction
        :param a_lat: acceleration in the lateral direction
        :param t_f: final step
        :param grid_shapes: shape of the grid
        """
        self.cc_static: pycrcc.CollisionChecker = pycrcc.CollisionChecker()
        self.cc_dynamic: pycrcc.CollisionChecker = pycrcc.CollisionChecker()
        self.grid_shapes = grid_shapes

        # only keep obstacles with possible collision by examining max acceleration
        init_pos = planning_config.p_initial
        init_vel = planning_config.v_initial

        a_max = max([a_lon, a_lat])
        extreme_pos = np.array([init_pos + a_max * t_f ** 2 / 2 + init_vel * t_f,
                                init_pos - a_max * t_f ** 2 / 2 + init_vel * t_f,
                                init_pos - a_max * t_f ** 2 / 2 - init_vel * t_f,
                                init_pos])
        collision_reachable_area_tmp = pycrcc.RectAABB(np.max(extreme_pos[:, 0]) - np.min(extreme_pos[:, 0]),
                                                       np.max(extreme_pos[:, 1]) - np.min(extreme_pos[:, 1]),
                                                       (np.max(extreme_pos[:, 0]) + np.min(extreme_pos[:, 0])) / 2,
                                                       (np.max(extreme_pos[:, 1]) + np.min(extreme_pos[:, 1])) / 2)

        collision_checker = collision_checker.window_query(collision_reachable_area_tmp)
        for obj in collision_checker.obstacles():
            if type(obj) == pycrcc.TimeVariantCollisionObject:
                self.cc_dynamic.add_collision_object(obj)

            else:
                self.cc_static.add_collision_object(obj)

        self.dx = dx
        self.dy = dy
        self.r_x = self.dx / 2
        self.r_y = self.dy / 2
        self.dx_div = 1.0 / self.dx
        self.dy_div = 1.0 / self.dy
        self.ll: Dict[int, np.ndarray] = ll
        self.ur: Dict[int, np.ndarray] = ur

    def occupancy_grid_at_step(self, step: int, translate_reachset: np.ndarray) -> np.ndarray:
        """
        Get occupancy matrix defined over regular grid with 0 = occupied, 1 = free space.

        :param step: step of evaluation
        :param translate_reachset: translation of reachable set
        """
        occupancy_grid = np.ones(self.grid_shapes[step], dtype=np.uint8)

        # origin of rect grid
        ur_translated = self.ur[step] + translate_reachset
        ll_translated = self.ll[step] + translate_reachset
        collision_reachable_area_tmp = pycrcc.RectAABB((ur_translated[0] - ll_translated[0]) / 2,
                                                       (ur_translated[1] - ll_translated[1]) / 2,
                                                       (ur_translated[0] + ll_translated[0]) / 2,
                                                       (ur_translated[1] + ll_translated[1]) / 2)

        static_obstacles: pycrcc.CollisionChecker = \
            self.cc_static.window_query(collision_reachable_area_tmp)
        for obs in static_obstacles.obstacles():
            occupancy_grid = self._add_obstacle_at_time_opencv(obs, occupancy_grid,
                                                               ur_translated=ur_translated,
                                                               ll_translated=ll_translated)

        cc_dynamic: pycrcc.CollisionChecker = \
            self.cc_dynamic.time_slice(step).window_query(collision_reachable_area_tmp)

        for obs in cc_dynamic.obstacles():
            occupancy_grid = self._add_obstacle_at_time_opencv(obs, occupancy_grid,
                                                               ur_translated=ur_translated,
                                                               ll_translated=ll_translated)

        return occupancy_grid.astype(dtype=bool)

    def _add_obstacle_at_time_opencv(self,
                                     collision_object: pycrcc.CollisionObject,
                                     occupancy_grid: np.ndarray,
                                     ll_translated: np.ndarray,
                                     ur_translated: np.ndarray) -> np.ndarray:
        """
        Fill the occupancy grid considering the shape of the collision object.
        """

        def fill_shape(occupancy_grid, collision_object):
            typ = type(collision_object)
            if typ == pycrcc.Polygon or typ == pycrcc.Triangle:
                vertices = np.array(collision_object.vertices())
                vertices = np.asarray(convert_cart2pixel_coordinates_c(vertices,
                                                                       ll_translated[0],
                                                                       ll_translated[1],
                                                                       self.dx_div,
                                                                       self.dy_div,
                                                                       vertices.shape[0]))
                vertices = vertices.reshape((-1, 1, 2))
                cv.fillPoly(occupancy_grid, [vertices], (0))

            elif typ == pycrcc.RectOBB:
                r_x = collision_object.r_x()
                r_y = collision_object.r_y()
                locx = collision_object.local_x_axis()
                locy = collision_object.local_y_axis()
                center = collision_object.center()
                vertices_cart = get_vertices_from_rect(center, r_x, r_y, locx, locy)
                vertices = np.asarray(convert_cart2pixel_coordinates_c(vertices_cart,
                                                                       ll_translated[0],
                                                                       ll_translated[1],
                                                                       self.dx_div,
                                                                       self.dy_div,
                                                                       4))
                vertices = vertices.reshape((-1, 1, 2))

                occupancy_grid = cv.fillPoly(occupancy_grid, [vertices], (0))

            elif typ == pycrcc.RectAABB:
                pt0 = convert_cart2pixel_coordinates(collision_object.min_x(), collision_object.min_y())
                pt1 = convert_cart2pixel_coordinates(collision_object.max_x(), collision_object.max_y())
                occupancy_grid = cv.rectangle(occupancy_grid, pt0, pt1, (0), -1)

            else:
                raise NotImplementedError('Type {} not Implemented'.format(typ))

            return occupancy_grid

        def convert_cart2pixel_coordinates(x: float, y: float) -> tuple:
            # x and y are switched for openCV!
            return (round((y - ll_translated[1]) * self.dy_div),
                    round((x - ll_translated[0]) * self.dx_div))

        occupancy_grid = fill_shape(occupancy_grid, collision_object)

        return occupancy_grid
