from typing import List

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.scenario import Scenario
from commonroad_reachset.data_structure.configuration import Configuration
from commonroad_reachset.data_structure.reach.reach_polygon import ReachPolygon
from shapely.geometry.polygon import Polygon


class PyCollisionChecker:
    def __init__(self, config: Configuration):
        self.config = config
        self.scenario: Scenario = config.scenario

    def collides_at_time_step(self, time_idx: int, rectangle: ReachPolygon) -> bool:
        """Checks for collision with obstacles in the scenario at time step."""

        list_polygons_collision_at_time_step = self.list_polygons_collision_at_time_step(time_idx)

        return self.rectangle_collides_with_obstacles(rectangle, list_polygons_collision_at_time_step)

    def list_polygons_collision_at_time_step(self, time_step: int):
        list_polygons = []

        list_occupancies = self.scenario.occupancies_at_time_step(time_step)
        for occ in list_occupancies:
            if isinstance(occ.shape, Rectangle):
                list_polygons.append(occ.shape.shapely_object)

        return list_polygons

    @staticmethod
    def rectangle_collides_with_obstacles(rectangle: ReachPolygon, list_obstacles: List[Polygon]):
        for obstacle in list_obstacles:
            if rectangle.intersects(obstacle):
                return True

        return False

    # def check_collision_and_split_rectangles(
    #         self, time_step: int, list_rectangles: List[ReachPolygon]) -> List[ReachPolygon]:
    #
    #     list_rectangles_collision_free = []
    #
    #     if not list_rectangles:
    #         return []
    #
    #     # 1. prepare obstacles
    #     list_polygons = self.list_polygons_collision_at_time_step(time_step)
    #
    #     if not len(list_polygons):
    #         return list_rectangles
    #
    #     # 2. check collision for each input rectangle
    #     for rectangle in list_rectangles:
    #         list_rectangles_collision_free += \
    #             self.split_rectangle(rectangle, list_polygons, self.config.reachable_set.radius_terminal_split)
    #
    #     return list_rectangles_collision_free
    #
    # def split_rectangle(self, rectangle: ReachPolygon, list_obstacles: List[Polygon], radius_terminal_squared: float):
    #
    #     # if does not collide, then return itself
    #     if not self.rectangle_collides_with_obstacles(rectangle, list_obstacles):
    #         return [rectangle]
    #
    #     # if the diagonal is smaller than the terminal threshold, stop
    #     elif rectangle.diagonal_squared < radius_terminal_squared:
    #         return []
    #
    #     # needs splitting
    #     else:
    #         rectangle_split_1, rectangle_split_2 = self.split_rectangle_into_two(rectangle)
    #         list_rectangles_split_1 = self.split_rectangle(rectangle_split_1, list_obstacles, radius_terminal_squared)
    #
    #         list_rectangles_split_2 = self.split_rectangle(rectangle_split_2, list_obstacles, radius_terminal_squared)
    #
    #         return list_rectangles_split_1 + list_rectangles_split_2
    #
    # @staticmethod
    # def split_rectangle_into_two(rectangle: ReachPolygon):
    #     if (rectangle.p_lon_max - rectangle.p_lon_min) > (rectangle.p_lat_max - rectangle.p_lat_min):
    #         rectangle_split_1 = ReachPolygon.from_rectangle_vertices(rectangle.p_lon_min, rectangle.p_lat_min,
    #                                                                  rectangle.p_lon_center, rectangle.p_lat_max)
    #
    #         rectangle_split_2 = ReachPolygon.from_rectangle_vertices(rectangle.p_lon_center, rectangle.p_lat_min,
    #                                                                  rectangle.p_lon_max, rectangle.p_lat_max)
    #     else:
    #         rectangle_split_1 = ReachPolygon.from_rectangle_vertices(rectangle.p_lon_min, rectangle.p_lat_min,
    #                                                                  rectangle.p_lon_max, rectangle.p_lat_center)
    #         rectangle_split_2 = ReachPolygon.from_rectangle_vertices(rectangle.p_lon_min, rectangle.p_lat_center,
    #                                                                  rectangle.p_lon_max, rectangle.p_lat_max)
    #
    #     return rectangle_split_1, rectangle_split_2
