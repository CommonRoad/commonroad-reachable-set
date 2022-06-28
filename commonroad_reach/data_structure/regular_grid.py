import pyximport
pyximport.install()

from typing import Tuple, Dict

import commonroad_dc.pycrcc as pycrcc
import cv2 as cv
import numpy as np

from commonroad_reach.data_structure.configuration import PlanningConfiguration
from commonroad_reach.utility.util_py_grid_online_reach import convert_cart2pixel_coordinates_c, get_vertices_from_rect

USE_COLLISION = True
CONVEX_SHAPE = True


class RegularGrid:
    def __init__(self, ll: Dict[int, np.ndarray], ur: Dict[int, np.ndarray], collision_checker: pycrcc.CollisionChecker,
                 dx: float, dy: float, planning_config: PlanningConfiguration, a_x: float, a_y: float, t_f: float,
                 grid_shapes: Dict[int, Tuple[int, int]]):
        """
        Class for computing uniformly spatial partitioned grid of obstacles
        :param collision_object_dict:
        :param ll: lower-left coordinates of reachable set at every time step
        :param ur: upper-right coordinates of reachable set at current time step
        :param collision_checker: collision checker for discretization queries
        :param dx: grid length x
        :param dy: grid length y
        :param params:
        :param dynamic_only: only consider dynamic obstacles
        """
        self.cc_static: pycrcc.CollisionChecker = pycrcc.CollisionChecker()
        self.cc_dynamic: pycrcc.CollisionChecker = pycrcc.CollisionChecker()
        self.grid_shapes = grid_shapes

        # filter irrelevant obstacles over complete time interval
        init_pos = planning_config.p_lon_lat_initial
        init_vel = planning_config.v_lon_lat_initial

        a_max = max([a_x, a_y])
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
        :return:
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
