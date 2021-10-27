from typing import List

import commonroad_dc.pycrcc as pycrcc
import numpy as np
import pycrreachset as reach
from commonroad.geometry.shape import Rectangle, ShapeGroup
from commonroad.scenario.obstacle import StaticObstacle
from commonroad.scenario.scenario import Scenario
from commonroad_dc.boundary import boundary
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object
from commonroad_reachset.data_structure.configuration import Configuration
from commonroad_reachset.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reachset.utility.geometry import create_aabb_from_coordinates


class CollisionChecker:
    """Collision checker for the reachable sets.

    This handles collision checks in both Cartesian and Curvilinear coordinate systems.
    """

    def __init__(self, config: Configuration):
        self.config = config
        self._cpp_collision_checker = None
        self._initialize_collision_checker()

    @property
    def collision_checker_cpp(self):
        return self._cpp_collision_checker

    def _initialize_collision_checker(self):
        """Initializes the collision checker based on the specified coordinate system"""
        # self._cpp_collision_checker = self._custom_collision_checker()

        if self.config.planning.coordinate_system == "CART":
            self._cpp_collision_checker = self._create_cartesian_collision_checker()

        elif self.config.planning.coordinate_system == "CVLN":
            self.list_vertices_polygons_static, \
            self.dict_time_to_list_vertices_polygons_dynamic = self.preprocess_obstacles()

            self._cpp_collision_checker = \
                reach.create_curvilinear_collision_checker(self.list_vertices_polygons_static,
                                                           self.dict_time_to_list_vertices_polygons_dynamic,
                                                           self.config.planning.CLCS,
                                                           self.config.vehicle.ego.radius_disc, 4)
        else:
            raise Exception("<CollisionChecker> Undefined coordinate system.")

    def _custom_collision_checker(self):
        if self.config.planning.coordinate_system == "CART":
            # cartesian collision checker
            collision_checker = pycrcc.CollisionChecker()
            collision_checker.add_collision_object(create_aabb_from_coordinates(0, -2, 150, 0))
            collision_checker.add_collision_object(create_aabb_from_coordinates(0, 8, 150, 10))
            collision_checker.add_collision_object(pycrcc.RectOBB(2.25, 1.0, 0.3, 65.0, 2.25))
            return collision_checker

        else:
            # curvilinear collision checker
            list_vertices_polygons = [np.array([[10, -2], [140, -2], [140, 0], [10, 0]]),
                                      np.array([[10, 8], [140, 8], [140, 10], [10, 10]])]

            collision_checker = reach.create_curvilinear_collision_checker(list_vertices_polygons, {},
                                                                           self.config.planning.CLCS,
                                                                           self.config.vehicle.radius_disc, 4)

            return collision_checker

    def _create_cartesian_collision_checker(self) -> pycrcc.CollisionChecker:
        """Creates a Cartesian collision checker.

        The collision checker is created by feeding in the scenario with road boundaries.
        """
        scenario_cc = self.create_scenario_with_road_boundaries(self.config)
        # parameter dictionary for inflation to consider the shape of the ego vehicle
        dict_param = {"minkowski_sum_circle": True,
                      "minkowski_sum_circle_radius": self.config.vehicle.ego.radius_disc,
                      "resolution": 5}
        collision_checker = self.create_cartesian_collision_checker_from_scenario(scenario_cc, params=dict_param)

        return collision_checker

    @staticmethod
    def create_scenario_with_road_boundaries(config: Configuration) -> Scenario:
        """Returns a scenario with obstacles in Cartesian coordinate system.

        Elements included: lanelet network, obstacles, road boundaries
        """
        scenario: Scenario = config.scenario
        scenario_cc = Scenario(scenario.dt, scenario.scenario_id)
        # add lanelet network
        scenario_cc.add_objects(scenario.lanelet_network)

        # add obstacles
        if config.reachable_set.consider_traffic:
            scenario_cc.add_objects(scenario.obstacles)

        # add road boundary static object
        object_road_boundary, _ = boundary.create_road_boundary_obstacle(scenario_cc, method="triangulation")
        scenario_cc.add_objects(object_road_boundary)

        return scenario_cc

    @staticmethod
    def create_cartesian_collision_checker_from_scenario(scenario, params):
        collision_checker = pycrcc.CollisionChecker()

        # dynamic obstacles
        for obs in scenario.dynamic_obstacles:
            collision_checker.add_collision_object(create_collision_object(obs, params))

        # static obstacles
        shape_group = pycrcc.ShapeGroup()
        for obs in scenario.static_obstacles:
            collision_object = create_collision_object(obs, params)

            if isinstance(collision_object, pycrcc.ShapeGroup):
                for shape in collision_object.unpack():
                    shape_group.add_shape(shape)

            else:
                shape_group.add_shape(collision_object)

        collision_checker.add_collision_object(shape_group)

        return collision_checker

    def _create_curvilinear_collision_checker(self) -> pycrcc.CollisionChecker:
        """Creates a Curvilinear collision checker.

        The collision checker is created by adding a shape group containing occupancies of all static obstacles, and a
        time variant object containing shape groups of occupancies of all dynamic obstacles at different time steps.
        """
        scenario = self.config.scenario
        # static obstacles
        list_obstacles_static = self.retrieve_static_obstacles(scenario, self.config.reachable_set.consider_traffic)
        list_vertices_polygons_static = self.obtain_vertices_of_polygons_for_static_obstacles(list_obstacles_static)

        # dynamic obstacles
        list_obstacles_dynamic = scenario.dynamic_obstacles
        dict_time_to_list_vertices_polygons_dynamic = \
            self.obtain_vertices_of_polygons_for_dynamic_obstacles(list_obstacles_dynamic)

        return reach.create_curvilinear_collision_checker(list_vertices_polygons_static,
                                                          dict_time_to_list_vertices_polygons_dynamic,
                                                          self.config.planning.CLCS,
                                                          self.config.vehicle.radius_disc, 4)

    def preprocess_obstacles(self):
        """Creates a Curvilinear collision checker.

        The collision checker is created by adding a shape group containing occupancies of all static obstacles, and a
        time variant object containing shape groups of occupancies of all dynamic obstacles at different time steps.
        """
        scenario = self.config.scenario
        # static obstacles
        list_obstacles_static = self.retrieve_static_obstacles(scenario, self.config.reachable_set.consider_traffic)
        list_vertices_polygons_static = self.obtain_vertices_of_polygons_for_static_obstacles(list_obstacles_static)

        # dynamic obstacles
        list_obstacles_dynamic = scenario.dynamic_obstacles
        dict_time_to_list_vertices_polygons_dynamic = \
            self.obtain_vertices_of_polygons_for_dynamic_obstacles(list_obstacles_dynamic)

        return list_vertices_polygons_static, dict_time_to_list_vertices_polygons_dynamic

    @staticmethod
    def retrieve_static_obstacles(scenario: Scenario, consider_traffic: bool) -> List[StaticObstacle]:
        """Returns the list of static obstacles.

        The static obstacles include static road traffic and road boundaries.
        """
        list_obstacles_static = []

        if consider_traffic:
            list_obstacles_static += scenario.static_obstacles

        # add road boundary
        object_road_boundary, _ = boundary.create_road_boundary_obstacle(scenario, method="triangulation")
        list_obstacles_static.append(object_road_boundary)

        return list_obstacles_static

    @staticmethod
    def obtain_vertices_of_polygons_for_static_obstacles(list_obstacles_static):
        list_vertices_polygons_static = []

        for obstacle in list_obstacles_static:
            # time_step = obstacle.initial_state.time_step
            occupancy = obstacle.occupancy_at_time(0)
            shape = occupancy.shape

            if isinstance(shape, Rectangle):
                list_vertices_polygons_static.append(shape.vertices)

            elif isinstance(shape, ShapeGroup):
                for shape in shape.shapes:
                    list_vertices_polygons_static.append(shape.vertices)

        return list_vertices_polygons_static

    def obtain_vertices_of_polygons_for_dynamic_obstacles(self, list_obstacles_dynamic):
        time_start = self.config.planning.time_step_start
        time_end = time_start + self.config.planning.time_steps_computation + 1

        dict_time_to_list_vertices_polygons_dynamic = {time_step: [] for time_step in range(time_start, time_end)}

        for time_step in range(time_start, time_end):
            list_vertices_polygons_dynamic = []

            for obstacle in list_obstacles_dynamic:
                occupancy = obstacle.occupancy_at_time(time_step)
                if not occupancy:
                    continue
                shape = occupancy.shape

                if isinstance(shape, Rectangle):
                    list_vertices_polygons_dynamic.append(shape.vertices)

                elif isinstance(shape, ShapeGroup):
                    for shape in shape.shapes:
                        list_vertices_polygons_dynamic.append(shape.vertices)

            dict_time_to_list_vertices_polygons_dynamic[time_step] += list_vertices_polygons_dynamic

        return dict_time_to_list_vertices_polygons_dynamic

    def collides_at_time_step(self, time_idx: int, input_rectangle: ReachPolygon) -> bool:
        """Checks for collision with obstacles in the scenario at time step.

        Note: creating a query windows significantly decreases computation time.
        """
        # convert to collision object
        rect_collision = self.convert_reach_polygon_to_collision_object(input_rectangle)

        # create a query window, decreases computation time
        collision_checker = self._cpp_collision_checker.window_query(rect_collision)

        # slice collision checker with time
        collision_checker_time_slice = collision_checker.time_slice(time_idx)

        # return collision result
        return collision_checker_time_slice.collide(rect_collision)

    @staticmethod
    def convert_reach_polygon_to_collision_object(input_rectangle: ReachPolygon) -> pycrcc.RectAABB:
        p_lon_min, p_lat_min, p_lon_max, p_lat_max = input_rectangle.bounds

        length = p_lon_max - p_lon_min
        width = p_lat_max - p_lat_min
        p_lon_center = (p_lon_max + p_lon_min) / 2.0
        p_lat_center = (p_lat_max + p_lat_min) / 2.0

        return pycrcc.RectAABB(length / 2, width / 2, p_lon_center, p_lat_center)
