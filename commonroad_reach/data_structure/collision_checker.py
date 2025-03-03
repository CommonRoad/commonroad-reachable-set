import logging
from typing import List, Dict
from shapely.geometry import Point

import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.boundary import boundary
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_object

import commonroad_reach.pycrreach as reach
from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon

from commonroad.geometry.shape import Rectangle, ShapeGroup, Polygon, Circle
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle
from commonroad.scenario.scenario import Scenario, LaneletNetwork

import commonroad_reach.utility.logger as util_logger

logger = logging.getLogger("REACH_LOGGER")


class CollisionChecker:
    """
    Collision checker using C++ backend.

    It handles collision checks in both Cartesian and curvilinear coordinate systems.
    """

    def __init__(self, config: Configuration):
        self.config = config
        self._initialize()

        logger.debug("CollisionChecker initialized.")

    def _initialize(self):
        if self.config.planning.coordinate_system == "CART":
            self.cpp_collision_checker = self._create_cartesian_collision_checker()

        elif self.config.planning.coordinate_system == "CVLN":
            self.cpp_collision_checker = self._create_curvilinear_collision_checker()

        else:
            message = "Undefined coordinate system."
            util_logger.print_and_log_error(logger, message)
            raise Exception(message)

    @staticmethod
    def create_scenario_with_road_boundaries(config: Configuration) -> Scenario:
        """
        Returns a scenario with obstacles in the Cartesian coordinate system.

        Elements included: lanelet network, obstacles, road boundaries.
        """
        scenario: Scenario = config.scenario
        lanelet_network: LaneletNetwork = config.planning.lanelet_network

        scenario_cc = Scenario(scenario.dt, scenario.scenario_id)

        # add lanelet network
        scenario_cc.add_objects(lanelet_network)

        # add obstacles
        if config.reachable_set.consider_traffic:
            scenario_cc.add_objects(scenario.obstacles)

        # add road boundary static object
        object_road_boundary, _ = boundary.create_road_boundary_obstacle(scenario_cc, method="obb_rectangles",
                                                                         width=2e-3)
        scenario_cc.add_objects(object_road_boundary)

        return scenario_cc

    @staticmethod
    def create_cartesian_collision_checker_from_scenario(scenario: Scenario, params: Dict) -> pycrcc.CollisionChecker:
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

    def _create_cartesian_collision_checker(self) -> pycrcc.CollisionChecker:
        """
        Creates a Cartesian collision checker.

        The collision checker is created by adding a shape group containing occupancies of all static obstacles, and a
        time variant object containing shape groups of occupancies of all dynamic obstacles at different steps.
        """
        scenario = self.config.scenario
        lanelet_network = self.config.planning.lanelet_network

        # static obstacles
        list_obstacles_static = self.retrieve_static_obstacles(scenario, lanelet_network,
                                                               self.config.reachable_set.consider_traffic)
        list_vertices_polygons_static = self.obtain_vertices_of_polygons_from_static_obstacles(list_obstacles_static)

        # dynamic obstacles
        list_obstacles_dynamic = scenario.dynamic_obstacles
        dict_time_to_list_vertices_polygons_dynamic = \
            self.obtain_vertices_of_polygons_for_dynamic_obstacles(list_obstacles_dynamic,
                                                                   self.config.reachable_set.consider_traffic)

        return reach.create_cartesian_collision_checker(list_vertices_polygons_static,
                                                        dict_time_to_list_vertices_polygons_dynamic,
                                                        self.config.vehicle.ego.radius_inflation, 4)

    def _create_curvilinear_collision_checker(self) -> pycrcc.CollisionChecker:
        """
        Creates a curvilinear collision checker.

        The collision checker is created by adding a shape group containing occupancies of all static obstacles, and a
        time variant object containing shape groups of occupancies of all dynamic obstacles at different steps.
        """
        scenario = self.config.scenario
        lanelet_network = self.config.planning.lanelet_network

        # static obstacles
        list_obstacles_static = self.retrieve_static_obstacles(scenario, lanelet_network,
                                                               self.config.reachable_set.consider_traffic)
        list_vertices_polygons_static = self.obtain_vertices_of_polygons_from_static_obstacles(list_obstacles_static)

        # dynamic obstacles
        list_obstacles_dynamic = scenario.dynamic_obstacles
        dict_time_to_list_vertices_polygons_dynamic = \
            self.obtain_vertices_of_polygons_for_dynamic_obstacles(list_obstacles_dynamic,
                                                                   self.config.reachable_set.consider_traffic)

        return reach.create_curvilinear_collision_checker(list_vertices_polygons_static,
                                                          dict_time_to_list_vertices_polygons_dynamic,
                                                          self.config.planning.CLCS,
                                                          self.config.vehicle.ego.radius_inflation, 4,
                                                          self.config.reachable_set.rasterize_obstacles,
                                                          self.config.reachable_set.rasterize_exclude_static)

    @staticmethod
    def retrieve_static_obstacles(scenario: Scenario, lanelet_network: LaneletNetwork, consider_traffic: bool) \
            -> List[StaticObstacle]:
        """
        Returns the list of static obstacles.

        The static obstacles include static road traffic (obtained from scenario object) and road boundaries (obtained
        from lanelet_network object).
        """
        list_obstacles_static = []

        if consider_traffic:
            list_obstacles_static += scenario.static_obstacles

        # compute road boundary
        scenario_cc = Scenario(scenario.dt, scenario.scenario_id)
        # add lanelet network
        scenario_cc.add_objects(lanelet_network)
        object_road_boundary, _ = boundary.create_road_boundary_obstacle(scenario_cc, method="obb_rectangles",
                                                                         width=2e-3)
        list_obstacles_static.append(object_road_boundary)

        return list_obstacles_static

    @staticmethod
    def obtain_vertices_of_polygons_from_static_obstacles(list_obstacles_static: List[StaticObstacle]):
        """
        Returns the vertices of polygons from a given list of static obstacles.
        """
        list_vertices_polygons_static = []

        for obstacle in list_obstacles_static:
            occupancy = obstacle.occupancy_at_time(0)
            shape = occupancy.shape

            if isinstance(shape, Rectangle):
                list_vertices_polygons_static.append(shape.vertices)

            elif isinstance(shape, ShapeGroup):
                for shape in shape.shapes:
                    list_vertices_polygons_static.append(shape.vertices)

        return list_vertices_polygons_static

    def obtain_vertices_of_polygons_for_dynamic_obstacles(self, list_obstacles_dynamic: List[DynamicObstacle],
                                                          consider_traffic: bool):
        """
        Returns the vertices of polygons from a given list of dynamic obstacles.
        """
        step_start = self.config.planning.step_start
        step_end = step_start + self.config.planning.steps_computation + 1

        dict_time_to_list_vertices_polygons_dynamic = {step: [] for step in range(step_start, step_end)}

        if consider_traffic:
            for step in range(step_start, step_end):
                list_vertices_polygons_dynamic = []
                time_step = step * round(self.config.planning.dt / self.config.scenario.dt)
                for obstacle in list_obstacles_dynamic:
                    occupancy = obstacle.occupancy_at_time(time_step)
                    if not occupancy:
                        continue
                    shape = occupancy.shape

                    if isinstance(shape, Rectangle) or isinstance(shape, Polygon):
                        list_vertices_polygons_dynamic.append(shape.vertices)
                    elif isinstance(shape, Circle):
                        list_vertices_polygons_dynamic.append(Polygon(shape.shapely_object.buffer(0.01).exterior).vertices)
                    elif isinstance(shape, ShapeGroup):
                        for shape in shape.shapes:
                            list_vertices_polygons_dynamic.append(shape.vertices)
                    else:
                        raise NotImplementedError(f"collision_checker.py: "
                                                  f"obstacle occupancy shape {shape} is not supported!")

                dict_time_to_list_vertices_polygons_dynamic[step] += list_vertices_polygons_dynamic

        return dict_time_to_list_vertices_polygons_dynamic

    def collides_at_step(self, step: int, input_rectangle: ReachPolygon) -> bool:
        """
        Returns true if the input rectangle collides with obstacles in the scenario at the given step.

        Creating a query windows significantly decreases computation time.
        """
        # convert to collision object
        rect_collision = self.convert_reach_polygon_to_collision_object(input_rectangle)

        # create a query window, decreases computation time
        collision_checker = self.cpp_collision_checker.window_query(rect_collision)

        # slice collision checker with time
        collision_checker_time_slice = collision_checker.time_slice(step)

        # return collision result
        return collision_checker_time_slice.collide(rect_collision)

    @staticmethod
    def convert_reach_polygon_to_collision_object(input_rectangle: ReachPolygon) -> pycrcc.RectAABB:
        """
        Converts a reach polygon object into a collision object of CommonRoad Drivability Checker.
        """
        p_lon_min, p_lat_min, p_lon_max, p_lat_max = input_rectangle.bounds

        length = p_lon_max - p_lon_min
        width = p_lat_max - p_lat_min
        p_lon_center = (p_lon_max + p_lon_min) / 2.0
        p_lat_center = (p_lat_max + p_lat_min) / 2.0

        return pycrcc.RectAABB(length / 2, width / 2, p_lon_center, p_lat_center)
