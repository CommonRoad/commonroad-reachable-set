import commonroad_dc.pycrcc as pycrcc
from pruningPy.graph_reach.util_reach import convert_cart2pixel_coordinates_c, get_vertices_from_rect

from typing import Tuple
from pruningPy.graph_reach.utils import *
from collections import defaultdict
import time
import cv2 as cv

USE_COLLISION = True
CONVEX_SHAPE = True


class ObstacleRegularGrid:
    def __init__(self, ll: List[Tuple[float]], ur: List[Tuple[float]], collision_checker: pycrcc.CollisionChecker, dx, dy, reach_params, a_x, a_y, t_f):
        """
        Class for computing uniformly spatial partinioned grid of obstacles
        :param collision_object_dict:
        :param ll: lower-left coordinates of reachable set at every time step
        :param ur: upper-right of reachable set at current time step
        :param scenario:
        :param dx: grid length x
        :param dy: grid length y
        :param params:
        :param dynamic_only: only consider dynamic obstacles
        """
        self.cc_static: pycrcc.CollisionChecker = pycrcc.CollisionChecker()
        self.cc_dynamic: pycrcc.CollisionChecker = pycrcc.CollisionChecker()

        # filter irrelevant obstacles over complete time interval
        init_pos = reach_params['initial_state'].position
        init_vel = reach_params['initial_state'].velocity
        init_ori = reach_params['initial_state'].orientation
        a_max = max([a_x,a_y])
        extreme_pos = np.array([init_pos + a_max * t_f ** 2 / 2 + init_vel * np.array([np.cos(init_ori),np.sin(init_ori)]) * t_f,
                        init_pos - a_max * t_f ** 2 / 2 + init_vel * np.array([np.cos(init_ori),np.sin(init_ori)]) * t_f,
                        init_pos - a_max * t_f ** 2 / 2 - init_vel * np.array([np.cos(init_ori),np.sin(init_ori)]) * t_f,
                        init_pos])
        collision_reachable_area_tmp = pycrcc.RectAABB(np.max(extreme_pos[:,0])-np.min(extreme_pos[:,0]),
                                                       np.max(extreme_pos[:,1])-np.min(extreme_pos[:,1]),
                                                       (np.max(extreme_pos[:,0])+np.min(extreme_pos[:,0]))/2,
                                                       (np.max(extreme_pos[:,1])+np.min(extreme_pos[:,1]))/2)

        collision_checker = collision_checker.window_query(collision_reachable_area_tmp)
        for obj in collision_checker.obstacles():
            if type(obj) == pycrcc.TimeVariantCollisionObject:
                self.cc_dynamic.add_collision_object(obj)
            else:
                self.cc_static.add_collision_object(obj)



        self.dx = np.array(dx)
        self.dy = np.array(dy)
        self.dx_div = 1.0 / self.dx
        self.dy_div = 1.0 / self.dy
        self.ll = np.array(ll)
        self.ur = np.array(ur)
        self.occupied_grid_obs_static = defaultdict(None)

    # TODO: compute obstacle grids offline in advacne (for optim)

    # def initialize_collision_objects(self, cc_static: collision.CollisionChecker, cc_dynamic: collision.CollisionChecker, params=None) -> Dict:
    #     """
    #     Initialize collision objects for rasterization
    #     :param scenario:
    #     :param params:
    #     :return:
    #     """
    #     # both sub-dicts contain ostacle_ids and respective collision objects
    #     obstacle_dict: Dict[ObstacleRole, Dict] = {ObstacleRole.STATIC: defaultdict(None),   # {obstacle_id: collision objects, ...}
    #                                                ObstacleRole.DYNAMIC: defaultdict(list)}  # {obstacle_id: [collision object_t0, ..., collision object_tN],...}
    #     # static objects onl
    #     # for obs in scenario.static_obstacles:
    #     obstacle_dict['static'] = cc_static
    #
    #     # pre-compute static objects
    #     # static_occ_grid = dict()
    #     # for t in range(0, params.t_f):
    #     #     static_occ_grid[t] = self._rasterize_cc(cc_static, self.ll[t], self.ur[t])
    #     #
    #     # for obs in scenario.obj:
    #     #     obstacle_dict[ObstacleRole.DYNAMIC][obs.obstacle_id].append(create_collision_object(obs.occupancy_at_time(0).shape)) #occupancy at initial time step
    #     #     for occ in obs.prediction.occupancy_set:
    #     obstacle_dict['dynamic'] = cc_dynamic
    #
    #     return obstacle_dict

    # def update_dyn_collision_objects(self, new_traj_list: List[List]):
    #     """
    #     Update all dynamic objects with new trajectories
    #     :param new_traj_list:
    #     :return:
    #     """
    #     for i_obj, obj_timelist in enumerate(self.collision_object_dict[ObstacleRole.DYNAMIC]):
    #         for i_t, obj in enumerate(obj_timelist):
    #             new_center = new_traj_list[i_obj][i_t]
    #             obj.set_center(new_center[0],new_center[1])

    def _rasterize_cc(self, cc: pycrcc.CollisionChecker, ll: np.ndarray, ur: np.ndarray):
        width = (ur[0] - ll[0])/2
        height = (ur[1] - ll[1])/2
        cc = cc.window_query(pycrcc.RectAABB(width, height, center_x = ll[0] + width, center_y = ll[1] + height))

    def get_occupancy_grid_at_time(self, id_t_next: int, translate_reachset: np.ndarray, propagated_set: np.ndarray, sparse: bool = False):

        # id_t = id_t-1
        # 0 = occupied, 1 = free space

        self.nx = int(np.ceil((self.ur[id_t_next][0] - self.ll[id_t_next][0]) / self.dx - 0.00001))
        self.ny = int(np.ceil((self.ur[id_t_next][1] - self.ll[id_t_next][1]) / self.dy - 0.00001))
        self.r_x = self.dx / 2
        self.r_y = self.dy / 2

        occupancy_grid = np.ones([self.nx, self.ny], dtype=np.uint8)

        #origin of rect grid
        self.ur_translated = self.ur[id_t_next] + translate_reachset
        self.ll_translated = self.ll[id_t_next] + translate_reachset
        self.collision_reachable_area_tmp = pycrcc.RectAABB((self.ur_translated[0] - self.ll_translated[0]) / 2,
                                                                 (self.ur_translated[1] - self.ll_translated[1]) / 2,
                                                                 (self.ur_translated[0] + self.ll_translated[0]) / 2,
                                                                 (self.ur_translated[1] + self.ll_translated[1]) / 2)

        if id_t_next not in self.occupied_grid_obs_static:
            static_obstacles: pycrcc.CollisionChecker = self.cc_static.window_query(self.collision_reachable_area_tmp)
            for obs in static_obstacles.obstacles():
                # occupancy_grid = self._add_obstacle_at_time(obs, id_t_next, occupancy_grid, translate_reachset, propagated_set) #TODO: delete
                occupancy_grid = self._add_obstacle_at_time_opencv(obs, id_t_next, occupancy_grid, translate_reachset, propagated_set)

            self.occupied_grid_obs_static[id_t_next] = occupancy_grid.copy()
        else:
            occupancy_grid = self.occupied_grid_obs_static[id_t_next].copy()

        cc_dynamic: pycrcc.CollisionChecker = self.cc_dynamic.time_slice(id_t_next).window_query(self.collision_reachable_area_tmp)

        for obs in cc_dynamic.obstacles():
            # occupancy_grid = self._add_obstacle_at_time(obs, id_t_next, occupancy_grid, translate_reachset, propagated_set) #TODO: delete
            occupancy_grid = self._add_obstacle_at_time_opencv(obs, id_t_next, occupancy_grid, translate_reachset, propagated_set)

        # print(occupancy_grid)
        return occupancy_grid.astype(dtype=bool), self.occupied_grid_obs_static[id_t_next]

    def _add_obstacle_at_time_opencv(self, collision_object: pycrcc.CollisionObject, id_t: int, occupancy_grid: np.ndarray, translate_reachset: np.ndarray, propagated_set) -> np.ndarray:
        def convert_cart2pixel_coordinates(cart: np.ndarray) -> np.ndarray:
            # x and y are switched for openCV!
            arr = np.array([np.round((cart[:,1] - self.ll_translated[1]) * self.dy_div),
                             np.round((cart[:,0] - self.ll_translated[0]) * self.dx_div)], dtype= np.int32)
            return arr.transpose()

        # def get_vertices():

        def fill_shape(occupancy_grid, collision_object):
            typ = type(collision_object)
            if typ == pycrcc.Polygon or typ == pycrcc.Triangle:
                vertices = np.array(collision_object.vertices())
                # vertices = convert_cart2pixel_coordinates(vertices)
                vertices = np.asarray(convert_cart2pixel_coordinates_c(vertices, self.ll_translated[0], self.ll_translated[1], self.dx_div, self.dy_div, vertices.shape[0]))
                vertices = vertices.reshape((-1,1,2))
                cv.fillPoly(occupancy_grid, [vertices], (0))
            elif typ == pycrcc.RectOBB:
                r_x = collision_object.r_x()
                r_y = collision_object.r_y()
                locx = collision_object.local_x_axis()
                locy = collision_object.local_y_axis()
                center = collision_object.center()
                vertices_cart = get_vertices_from_rect(center, r_x, r_y, locx, locy)
                # vertices_2 = convert_cart2pixel_coordinates(vertices_cart)
                vertices = np.asarray(convert_cart2pixel_coordinates_c(vertices_cart, self.ll_translated[0], self.ll_translated[1], self.dx_div, self.dy_div, 4))
                vertices = vertices.reshape((-1, 1, 2))

                occupancy_grid = cv.fillPoly(occupancy_grid, [vertices], (0))
            elif typ == pycrcc.RectAABB:
                pt0 = tuple(convert_cart2pixel_coordinates(np.array([[collision_object.min_x(),
                                                                      collision_object.min_y()], ])).flatten().tolist())
                pt1 = tuple(convert_cart2pixel_coordinates(np.array([[collision_object.max_x(),
                                                                      collision_object.max_y()], ])).flatten().tolist())
                occupancy_grid = cv.rectangle(occupancy_grid, pt0, pt1, (0), -1)
            else:
                raise NotImplementedError('Type {} not Implemented'.format(typ))

            return occupancy_grid

        # vertices = np.array([[10, 10], [80, 40], [10, 90]], np.int32)
        # vertices = vertices.reshape((-1, 1, 2))
        width = self.ur_translated[0] - self.ll_translated[0]
        height = self.ur_translated[1] - self.ll_translated[1]
        # occupancy_grid_tmp = np.array(occupancy_grid, dtype=np.uint8)
        occupancy_grid = fill_shape(occupancy_grid, collision_object)

        return occupancy_grid

    def _add_obstacle_at_time(self, collision_object: pycrcc.CollisionObject, id_t: int, occupancy_grid: np.ndarray, translate_reachset: np.ndarray, propagated_set) -> np.ndarray:
        """
        Perform sampling by substituting the polygon with a regular grid of
        sample points within it. The distance between the sample points is
        given by x_interval and y_interval.
        :param delta_pos: translate obstacle by this vector
        """
        # t1=time.time()

        # get obstacle bounds
        # bvh=collision_object.get_bounding_volume()
        # ll_obs = [bvh.min_x(), bvh.min_y()]
        # ur_obs = [bvh.max_x(), bvh.max_y()]
        ll_obs, ur_obs = self._get_bounding_box(collision_object)

        # t1=time.time()
        # raster = rasterize([shape._shapely_polygon], out_shape=[self.max_xi,self.max_yi], default_value=1)
        # print('1',time.time()-t1)
        # t1 = time.time()
        # raster = rasterize([shape._shapely_polygon], out_shape=[self.max_xi, self.max_yi], default_value=1, all_touched=True)
        # print('6', time.time() - t1)
        # print(time.time() - t1, 't2')
        # t1 = time.time()
        # # print('OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOL')
        low_ix = int(np.floor((ll_obs[0] - self.ll_translated[0]) * self.dx_div + 0.0001))
        upp_ix = int(np.ceil((ur_obs[0] - self.ll_translated[0]) * self.dx_div - 0.0001))
        low_iy = int(np.floor((ll_obs[1] - self.ll_translated[1]) * self.dy_div + 0.0001))
        upp_iy = int(np.ceil((ur_obs[1] - self.ll_translated[1]) * self.dy_div - 0.0001))

        self.ll_translated_05 = np.empty_like(self.ll_translated)
        self.ll_translated_05[0] = self.ll_translated[0] + self.r_x
        self.ll_translated_05[1] = self.ll_translated[1] + self.r_y

        # print(time.time() - t1,'t3')
        #
        #
        # for x in range(max(low_ix,self.orig_xi), min(upp_ix, self.max_xi)):
        #     for y in range(max(low_iy,self.orig_yi), min(upp_iy, self.max_yi)):
        #         if shape.contains_point([(x+0.5)* self.dx, (y+0.5) * self.dy]):
        #             # if x - self.orig_xi >=0 and x - self.orig_xi < occupancy_grid.shape[0] and y - self.orig_yi>=0 and y - self.orig_yi < occupancy_grid.shape[1]:
        #             occupancy_grid[x - self.orig_xi, y - self.orig_yi] = 0
        # print('2',time.time() - t1)
        # t1 = time.time()
        t1 = time.time()
        if USE_COLLISION==False:
            # for x in range(max(0,low_ix), min(upp_ix, self.nx)):
            #     next=1
            #     for y in range(min(low_iy,0), min(upp_iy, self.ny)):
            #         if shape.contains_point([x * self.dx + self.ll_translated_05[0], y * self.dy + self.ll_translated_05[1]]):
            #             # if x - self.orig_xi >=0 and x - self.orig_xi < occupancy_grid.shape[0] and y - self.orig_yi>=0 and y - self.orig_yi < occupancy_grid.shape[1]:
            #             occupancy_grid[x, y] = 0
            #             next=0
            #             break
            #     if next==0:
            #         y2=y # in case following loop is not entered
            #         for y2 in range(min(upp_iy, self.ny), y, -1):
            #             if shape.contains_point([x * self.dx + self.ll_translated_05[0], y * self.dy + self.ll_translated_05[1]]):
            #                 # if x - self.orig_xi >=0 and x - self.orig_xi < occupancy_grid.shape[0] and y - self.orig_yi>=0 and y - self.orig_yi < occupancy_grid.shape[1]:
            #                 occupancy_grid[x, y] = 0
            #                 break
            #         # if min(upp_iy, self.max_yi)!=y:
            #         occupancy_grid[x, y:y2] = 0
            #
            for x in range(max(0, low_ix), min(upp_ix, self.nx)):
                no_coll = 1
                for y in range(max(low_iy, 0), min(upp_iy, self.ny)):
                    # if propagated_set[x,y] and occupancy_grid[x, y] == 1:
                    # if occupancy_grid[x, y] == 1:
                    if collision_object.collide(pycrcc.Point(x * self.dx + self.ll_translated_05[0], y * self.dy + self.ll_translated_05[1])):
                        # if car_collision_object.collide(collision.RectAABB(r_x, r_y, x * self.dx + self.ll_translated_05[0],y * self.dy + self.ll_translated_05[1])):
                        # if x - self.orig_xi >=0 and x - self.orig_xi < occupancy_grid.shape[0] and y - self.orig_yi>=0 and y - self.orig_yi < occupancy_grid.shape[1]:
                        occupancy_grid[x, y] = 0
                        no_coll = 0
                        break
                if no_coll == 0:
                    y2 = y  # in case following loop is not entered
                    for y2 in range(min(upp_iy, self.ny - 1), y, -1):
                        # if propagated_set[x, y2] and occupancy_grid[x, y2] == 1:
                        # if occupancy_grid[x, y2] == 1:
                        if collision_object.collide(pycrcc.Point(x * self.dx + self.ll_translated_05[0], y2 * self.dy + self.ll_translated_05[1])):
                            # if car_collision_object.collide(collision.RectAABB(r_x, r_y, x * self.dx + self.ll_translated_05[0], y * self.dy + self.ll_translated_05[1])):
                            # if x - self.orig_xi >=0 and x - self.orig_xi < occupancy_grid.shape[0] and y - self.orig_yi>=0 and y - self.orig_yi < occupancy_grid.shape[1]:
                            occupancy_grid[x, y2] = 0
                            break
                    # if min(upp_iy, self.max_yi)!=y:
                    occupancy_grid[x, y:y2] = 0
        else:
            for x in range(max(0,low_ix), min(upp_ix, self.nx)):
                if CONVEX_SHAPE is True:
                    no_coll = 1
                    for y in range(max(low_iy, 0), min(upp_iy, self.ny)):
                        #if propagated_set[x,y] and occupancy_grid[x, y] == 1:
                        #if occupancy_grid[x, y] == 1:
                        if collision_object.collide(pycrcc.RectAABB(self.r_x, self.r_y, x * self.dx + self.ll_translated_05[0], y * self.dy + self.ll_translated_05[1])):
                            # if car_collision_object.collide(collision.RectAABB(r_x, r_y, x * self.dx + self.ll_translated_05[0],y * self.dy + self.ll_translated_05[1])):
                            # if x - self.orig_xi >=0 and x - self.orig_xi < occupancy_grid.shape[0] and y - self.orig_yi>=0 and y - self.orig_yi < occupancy_grid.shape[1]:
                            occupancy_grid[x, y] = 0
                            no_coll = 0
                            break
                    if no_coll == 0:
                        y2 = y  # in case following loop is not entered
                        for y2 in range(min(upp_iy, self.ny-1), y, -1):
                            #if propagated_set[x, y2] and occupancy_grid[x, y2] == 1:
                            #if occupancy_grid[x, y2] == 1:
                            if collision_object.collide(pycrcc.RectAABB(self.r_x, self.r_y, x * self.dx +self.ll_translated_05[0], y2 * self.dy + self.ll_translated_05[1])):
                            # if car_collision_object.collide(collision.RectAABB(r_x, r_y, x * self.dx + self.ll_translated_05[0], y * self.dy + self.ll_translated_05[1])):
                                # if x - self.orig_xi >=0 and x - self.orig_xi < occupancy_grid.shape[0] and y - self.orig_yi>=0 and y - self.orig_yi < occupancy_grid.shape[1]:
                                occupancy_grid[x, y2] = 0
                                break
                        # if min(upp_iy, self.max_yi)!=y:
                        occupancy_grid[x, y:y2] = 0
                else:  # CONVEX_SHAPE is False
                    for y in range(max(low_iy, 0), min(upp_iy, self.ny)):
                        if collision_object.collide(
                                pycrcc.RectAABB(self.r_x, self.r_y, x * self.dx + self.ll_translated_05[0],
                                                y * self.dy + self.ll_translated_05[1])):
                            occupancy_grid[x, y] = 0

        return occupancy_grid

    def _get_bounding_box(self, collision_object) -> Tuple[np.ndarray, np.ndarray]:
        """Compute bounding boxes and return: lower left and upper right"""
        if type(collision_object) == pycrcc.RectOBB:
            r_x = collision_object.r_x()
            r_y = collision_object.r_y()
            locx = np.array(collision_object.local_x_axis())
            locy = np.array(collision_object.local_y_axis())
            r_x2 = locx * r_x
            r_y2 = locy * r_y
            box = np.abs(r_x2) + np.abs(r_y2)
            center = np.array(collision_object.center())
            ll_obs = center - box
            ur_obs = center + box
        elif type(collision_object) == pycrcc.RectAABB:
            ll_obs = np.array([collision_object.min_x(), collision_object.min_y()])
            ur_obs = np.array([collision_object.max_x(), collision_object.max_y()])
        elif type(collision_object) == pycrcc.Polygon:
            vertices = np.array(collision_object.vertices())
            ll_obs = np.min(vertices,axis=0)
            ur_obs = np.max(vertices,axis=0)
        elif type(collision_object) == pycrcc.ShapeGroup:
            ll_obs = np.array([np.inf,np.inf])
            ur_obs = np.array([-np.inf, -np.inf])
            for shape in collision_object.unpack():
                ll_obs_tmp, ur_obs_tmp = self._get_bounding_box(shape)
                ll_obs = np.minimum(ll_obs_tmp, ll_obs)
                ur_obs = np.maximum(ur_obs_tmp, ur_obs)
        else:
            raise ValueError("No boundary computation for type {} implemented.".format(str(type(collision_object))))

        return ll_obs, ur_obs