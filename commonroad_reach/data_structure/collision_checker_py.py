import logging

logger = logging.getLogger(__name__)
from typing import List, Optional

import numpy as np
from commonroad.geometry.shape import Rectangle, Shape, Polygon, ShapeGroup
from commonroad.scenario.obstacle import StaticObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad_dc.boundary import boundary
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem

from commonroad_reach.data_structure.configuration import Configuration
from commonroad_reach.data_structure.reach.reach_polygon import ReachPolygon
from commonroad_reach.utility import geometry as util_geometry


class PyCollisionChecker:
    """Collision checker for the reachable sets with Python backend.

    It handles collision checks in both Cartesian and Curvilinear coordinate systems.
    """

    def __init__(self, config: Configuration):
        self.config: Configuration = config
        self.CLCS: CurvilinearCoordinateSystem = config.planning.CLCS
        self.scenario_cc: Optional[Scenario] = None

        radius_disc = self.config.vehicle.ego.radius_disc
        # used in minkowski sum operation to approximate the shape of the ego vehicle
        self.rectangle_for_inflation = Rectangle(length=radius_disc, width=radius_disc)
        self._initialize_collision_checker()

        logger.info("PyCollisionChecker initialized.")

    def _initialize_collision_checker(self):
        """Initializes the collision checker based on the specified coordinate system."""
        if self.config.planning.coordinate_system == "CART":
            self.scenario_cc = self._create_scenario_for_cartesian_collision_check(self.config)

        elif self.config.planning.coordinate_system == "CVLN":
            self.scenario_cc = self._create_scenario_for_curvilinear_collision_check(self.config)

        else:
            message = "Undefined coordinate system."
            logger.exception(message)
            raise Exception(message)

    @staticmethod
    def _create_scenario_for_cartesian_collision_check(config: Configuration):
        """Returns a scenario with obstacles in Cartesian coordinate system.

        Elements included: obstacles, road boundaries.
        """
        scenario: Scenario = config.scenario
        scenario_cc = Scenario(scenario.dt, scenario.scenario_id)

        # add lanelet network
        scenario_cc.add_objects(scenario.lanelet_network)

        list_obstacles = []
        # add obstacles
        if config.reachable_set.consider_traffic:
            list_obstacles += config.scenario.obstacles

        scenario_cc.add_objects(list_obstacles)

        # add road boundary
        object_road_boundary, _ = boundary.create_road_boundary_obstacle(scenario_cc, method="triangulation")
        scenario_cc.add_objects(object_road_boundary)

        return scenario_cc

    def _create_scenario_for_curvilinear_collision_check(self, config: Configuration):
        """Returns a scenario with obstacles in Curvilinear coordinate system.

        Elements included: obstacles, road boundaries.
        The obstacles (vehicles and road boundaries) are converted into Curvilinear coordinate system.
        """
        scenario: Scenario = config.scenario
        scenario_cc = Scenario(scenario.dt, scenario.scenario_id)

        # add lanelet network
        scenario_cc.add_objects(scenario.lanelet_network)

        list_obstacles_cart = []
        # add obstacles
        if config.reachable_set.consider_traffic:
            list_obstacles_cart += config.scenario.obstacles

        # add road boundary
        object_road_boundary, _ = boundary.create_road_boundary_obstacle(scenario_cc, method="triangulation")
        list_obstacles_cart.append(object_road_boundary)

        # convert obstacles into curvilinear coordinate system
        list_obstacles_cvln = self.convert_obstacles_to_curvilinear_coordinate_system(list_obstacles_cart)

        scenario_cc.add_objects(list_obstacles_cvln)

        return scenario_cc

    def convert_obstacles_to_curvilinear_coordinate_system(self, list_obstacles_cart):
        """Returns a list of obstacles converted into curvilinear coordinate system.

        Splitting obstacles in the Cartesian coordinate system into smaller rectangles (rasterization) reduces over-
        approximation in the curvilinear coordinate system, since they are converted into axis-aligned rectangles.
        """
        list_obstacles_static_cart = [obs for obs in list_obstacles_cart if isinstance(obs, StaticObstacle)]
        list_obstacles_static_cvln = self.convert_to_curvilinear_static_obstacles(list_obstacles_static_cart)

        # todo: implement this
        # list_obstacles_dynamic_cart = [obs for obs in list_obstacles_cart if isinstance(obs, DynamicObstacle)]
        # list_obstacles_dynamic_cvln = self.convert_to_curvilinear_dynamic_obstacles(list_obstacles_dynamic_cart, CLCS)
        # return list_obstacles_static_cvln + list_obstacles_dynamic_cvln

        return list_obstacles_static_cvln

    def convert_to_curvilinear_static_obstacles(self, list_obstacles_static_cart):
        """Converts a list of static obstacles to obstacle under Curvilinear coordinate system."""
        list_obstacles_static_cvln = []

        for obstacle in list_obstacles_static_cart:
            time_step_initial = obstacle.initial_state.time_step

            occupancy = obstacle.occupancy_at_time(time_step_initial)

            if isinstance(occupancy.shape, ShapeGroup):
                for shape in occupancy.shape.shapes:
                    shape_obstacle_cvln, position_cvln = self.convert_to_curvilinear_shape(shape)
                    if shape_obstacle_cvln is None or position_cvln is None:
                        continue

                    id_obstacle = self.config.scenario.generate_object_id()
                    type_obstacle = obstacle.obstacle_type

                    state_initial_obstacle_cvln = State(position=np.array([0, 0]), orientation=0.00,
                                                        time_step=time_step_initial)

                    # feed in the required components to construct a static obstacle
                    static_obstacle = StaticObstacle(id_obstacle, type_obstacle,
                                                     shape_obstacle_cvln, state_initial_obstacle_cvln)

                    list_obstacles_static_cvln.append(static_obstacle)

            elif isinstance(occupancy.shape, Shape):
                shape_obstacle_cvln, _ = self.convert_to_curvilinear_shape(occupancy.shape)

                id_obstacle = obstacle.obstacle_id
                type_obstacle = obstacle.obstacle_type
                position = obstacle.initial_state.position
                position_obstacle_cvln = self.CLCS.convert_to_curvilinear_coords(position[0], position[1])

                state_initial_obstacle_cvln = State(position=position_obstacle_cvln, orientation=0.00,
                                                    time_step=time_step_initial)

                # feed in the required components to construct a static obstacle
                static_obstacle = StaticObstacle(id_obstacle, type_obstacle,
                                                 shape_obstacle_cvln, state_initial_obstacle_cvln)

                list_obstacles_static_cvln.append(static_obstacle)

        return list_obstacles_static_cvln

    def convert_to_curvilinear_shape(self, shape: Shape):
        """Converts a rectangle or polygon to Curvilinear coordinate system."""

        if isinstance(shape, Rectangle):
            shape_inflated = util_geometry.minkowski_sum(ReachPolygon(shape.vertices),
                                                         ReachPolygon(self.rectangle_for_inflation.vertices))

            list_vertices_cvln = self.convert_to_curvilinear_vertices(shape_inflated.vertices)
            rectangle, position = self.create_rectangle_from_vertices(list_vertices_cvln)

            return rectangle, position

        elif isinstance(shape, Polygon):
            shape_inflated = util_geometry.minkowski_sum(ReachPolygon(shape.vertices),
                                                         ReachPolygon(self.rectangle_for_inflation.vertices))
            try:
                position = self.CLCS.convert_to_curvilinear_coords(shape.center[0], shape.center[1])
                list_vertices_cvln = self.convert_to_curvilinear_vertices(shape_inflated.vertices)
                polygon = self.create_polygon_from_vertices(list_vertices_cvln)

                return polygon, position

            except ValueError:
                return None, None

        else:
            return None

    def convert_to_curvilinear_vertices(self, list_vertices_cart):
        """Converts a list of vertices to Curvilinear coordinate system."""
        try:
            list_vertices_cvln = [self.CLCS.convert_to_curvilinear_coords(vertex[0], vertex[1])
                                  for vertex in list_vertices_cart]

        except ValueError:
            return []

        else:
            return list_vertices_cvln

    @staticmethod
    def create_rectangle_from_vertices(list_vertices):
        """Returns a rectangle and its position for the given list of vertices."""
        if not list_vertices:
            return None, None

        list_p_lon = [vertex[0] for vertex in list_vertices]
        list_p_lat = [vertex[1] for vertex in list_vertices]

        p_lon_min = min(list_p_lon)
        p_lat_min = min(list_p_lat)
        p_lon_max = max(list_p_lon)
        p_lat_max = max(list_p_lat)

        length = p_lon_max - p_lon_min
        width = p_lat_max - p_lat_min
        position = np.array([(p_lon_min + p_lon_max) / 2.0, (p_lat_min + p_lat_max) / 2.0])

        return Rectangle(length=length, width=width), position

    @staticmethod
    def create_polygon_from_vertices(list_vertices):
        """Returns a polygon and its position for the given list of vertices."""
        if not list_vertices:
            return None

        return Polygon(np.array(list_vertices))

    def collides_at_time_step(self, time_idx: int, rectangle: ReachPolygon) -> bool:
        """Checks for collision with obstacles in the scenario at time step."""
        list_polygons_collision_at_time_step = self.list_polygons_collision_at_time_step(time_idx)

        return self.rectangle_collides_with_obstacles(rectangle, list_polygons_collision_at_time_step)

    def list_polygons_collision_at_time_step(self, time_step: int):
        """Returns the list of polygons for collision check at the given time step."""
        list_polygons = []

        list_occupancies = self.scenario_cc.occupancies_at_time_step(time_step)
        for occ in list_occupancies:
            if isinstance(occ.shape, ShapeGroup):
                for shape in occ.shape.shapes:
                    list_polygons.append(shape.shapely_object)

            elif isinstance(occ.shape, Rectangle) or isinstance(occ.shape, Polygon):
                list_polygons.append(occ.shape.shapely_object)

        return list_polygons

    @staticmethod
    def rectangle_collides_with_obstacles(rectangle: ReachPolygon, list_polygons: List[Polygon]):
        """Returns true if the input rectangle collides with the input list of polygons"""
        for polygon in list_polygons:
            if rectangle.intersects(polygon):
                return True

        return False
