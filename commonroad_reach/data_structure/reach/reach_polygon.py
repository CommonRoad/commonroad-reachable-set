import logging
from abc import ABC
from typing import List, Tuple, Union

import numpy as np
from shapely import Point
from shapely.geometry import Polygon
import commonroad_reach.utility.logger as util_logger

logger = logging.getLogger("REACH_LOGGER")


class ReachPolygon(ABC):
    """
    Polygon class that constitutes reach nodes and position rectangles.

    .. note::
        - When used to represent a reach node, it is defined in the position-velocity domain, and can be used to
          represent a polygon in either the longitudinal or the lateral direction.
        - When used to represent a position rectangle, it is defined in the longitudinal/lateral position domain.
    """

    def __init__(self, list_vertices: list, fix_vertices=True):
        if isinstance(list_vertices, list):
            if len(list_vertices) < 3:
                message = "A polygon needs at least 3 vertices."
                util_logger.print_and_log_error(logger, message)
                raise Exception(message)

            # Shapely polygon requires identical initial and final vertices
            if fix_vertices and not np.allclose(list_vertices[0], list_vertices[-1]):
                list_vertices.append(list_vertices[0])

        self._shapely_polygon = Polygon(list_vertices)
        self._bounds = self._shapely_polygon.bounds

    def __repr__(self):
        return f"ReachPolygon({self._bounds[0]:.4}, {self._bounds[1]:.4}, {self._bounds[2]:.4}, {self._bounds[3]:.4})"

    def __str__(self):
        return f"{self._bounds}"

    @property
    def shapely_object(self) -> Polygon:
        return self._shapely_polygon

    @property
    def bounds(self) -> List:
        return self._bounds

    @property
    def p_min(self):
        """
        Minimum position in the position-velocity domain.
        """
        return self._bounds[0]

    @property
    def p_max(self):
        """
        Maximum position in the position-velocity domain.
        """
        return self._bounds[2]

    @property
    def v_min(self):
        """
        Minimum velocity in the position-velocity domain.
        """
        return self._bounds[1]

    @property
    def v_max(self):
        """
        Maximum velocity in the position-velocity domain.
        """
        return self._bounds[3]

    @property
    def p_lon_min(self):
        """
        Minimum longitudinal position in the position domain.
        """
        return self._bounds[0]

    @property
    def p_lon_max(self):
        """
        Maximum longitudinal position in the position domain.
        """
        return self._bounds[2]

    @property
    def p_lon_center(self):
        """
        Center longitudinal position in the position domain.
        """
        return (self.p_lon_min + self.p_lon_max) / 2

    @property
    def p_lat_min(self):
        """
        Minimum lateral position in the position domain.
        """
        return self._bounds[1]

    @property
    def p_lat_max(self):
        """
        Maximum lateral position in the position domain.
        """
        return self._bounds[3]

    @property
    def p_lat_center(self):
        """
        Center lateral position in the position domain.
        """
        return (self.p_lat_min + self.p_lat_max) / 2

    @property
    def diagonal_squared(self):
        """
        Square length of the diagonal of the position domain.
        """
        return (self.p_lon_max - self.p_lon_min) ** 2 + (self.p_lat_max - self.p_lat_min) ** 2

    @property
    def vertices(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """
        Returns the list of vertices of the polygon.
        """
        if isinstance(self._shapely_polygon, Polygon):
            list_x, list_y = self._shapely_polygon.exterior.coords.xy

        else:
            message = "Polygon type error."
            util_logger.print_and_log_error(logger, message)
            raise Exception(message)

        list_vertices = [vertex for vertex in zip(list_x, list_y)]

        return list_vertices[:-1]

    @property
    def is_empty(self):
        """
        True if the set of points in this polygon is empty, else False
        """
        return self._shapely_polygon.is_empty

    @property
    def convex_hull(self) -> Polygon:
        """
        The convex hull of the polygon
        """
        return self._shapely_polygon.convex_hull

    @property
    def area(self) -> float:
        """
        The area of the polygon
        """
        return self._shapely_polygon.area

    def intersects(self, other_polygon: "ReachPolygon") -> bool:
        """
        Returns True if geometries intersect, else False
        """
        return self._shapely_polygon.intersects(other_polygon._shapely_polygon)

    def contains(self, point: Point) -> bool:
        """
        Returns True if the geometry contains the other, else False
        """
        return self._shapely_polygon.contains(point)

    def clone(self, convexify: bool) -> "ReachPolygon":
        """
        Returns a cloned (and convexified) polygon.
        """
        if convexify:
            return ReachPolygon.from_polygon(self._shapely_polygon.convex_hull)

        else:
            return ReachPolygon(self.vertices)

    def intersect_halfspace(self, a: float, b: float, c: float) -> Union["ReachPolygon", None]:
        """
        Returns the intersection of the polygon and the halfspace specified in the form of ax + by <= c.
        """
        assert not (a == 0 and b == 0), "Halfspace parameters are not valid."

        polygon_halfspace = self.construct_halfspace_polygon(a, b, c, self._bounds)
        polygon_intersected = self._shapely_polygon.intersection(polygon_halfspace)

        if isinstance(polygon_intersected, Polygon) and not polygon_intersected.is_empty:
            return ReachPolygon.from_polygon(polygon_intersected)
        else:
            return None

    def intersection(self, other_polygon: "ReachPolygon") -> Union["ReachPolygon", None]:
        """
        Computes intersection of a reach polygon with another reach polygon.
        If polygons intersect, resulting intersection is returned as a reach polygon, else None.
        """
        polygon_intersected = self._shapely_polygon.intersection(other_polygon._shapely_polygon)

        if isinstance(polygon_intersected, Polygon) and not polygon_intersected.is_empty:
            return ReachPolygon.from_polygon(polygon_intersected)
        else:
            return None

    @classmethod
    def from_polygon(cls, polygon: Polygon) -> Union["ReachPolygon", None]:
        """
        Returns a polygon constructed from the vertices of the given polygon.
        """
        if polygon.is_empty:
            return None

        else:
            return ReachPolygon(cls.get_vertices(polygon))

    @staticmethod
    def from_rectangle_vertices(p_lon_min: float, p_lat_min: float,
                                p_lon_max: float, p_lat_max: float) -> "ReachPolygon":
        """
        Returns a polygon given the vertices of a rectangle.
        """
        list_vertices = [(p_lon_min, p_lat_min), (p_lon_max, p_lat_min),
                         (p_lon_max, p_lat_max), (p_lon_min, p_lat_max)]
        return ReachPolygon(list_vertices)

    @staticmethod
    def get_vertices(polygon: Polygon) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Returns the list of vertices of the polygon."""
        if isinstance(polygon, Polygon):
            list_x, list_y = polygon.exterior.coords.xy

        else:
            message = "Polygon type error."
            util_logger.print_and_log_error(logger, message)
            raise Exception(message)

        list_vertices = [vertex for vertex in zip(list_x, list_y)]

        return list_vertices

    @staticmethod
    def construct_halfspace_polygon(a: float, b: float, c: float,
                                    bounds_polygon: Tuple[float, float, float, float]) -> Polygon:
        """
        Returns a polygon representing the halfspace.

        .. note::
            **General case:** First compute two arbitrary vertices that are far away from the x boundary,
            then compute the slope of the vector that is perpendicular to the vector connecting these two points to
            look for remaining two vertices required for the polygon construction.
        """
        x_min, y_min, x_max, y_max = bounds_polygon
        dist_diagonal = ((x_max - x_min) ** 2 + (y_max - y_min) ** 2) ** 0.5
        margin = 10

        list_vertices = []

        if b == 0:
            # vertical
            if a > 0:  # x <= c/a
                list_vertices.append([c / a, y_min - margin])
                list_vertices.append([c / a, y_max + margin])
                list_vertices.append([x_min - margin, y_max + margin])
                list_vertices.append([x_min - margin, y_min - margin])

            else:  # x >= c/a
                list_vertices.append([c / a, y_min - margin])
                list_vertices.append([c / a, y_max + margin])
                list_vertices.append([x_max + margin, y_max + margin])
                list_vertices.append([x_max + margin, y_min - margin])

        elif a == 0:
            # horizontal
            if b > 0:  # by <= c
                list_vertices.append([x_min - margin, c / b])
                list_vertices.append([x_max + margin, c / b])
                list_vertices.append([x_max + margin, y_min - margin])
                list_vertices.append([x_min - margin, y_min - margin])

            else:  # by <= c
                list_vertices.append([x_min - margin, c / b])
                list_vertices.append([x_max + margin, c / b])
                list_vertices.append([x_max + margin, y_max + margin])
                list_vertices.append([x_min - margin, y_max + margin])

        else:
            # general case
            margin = 100

            for x in [x_min - margin, x_max + margin]:
                y = (-a * x + c) / b

                list_vertices.append([x, y])

            vertex1 = list_vertices[0]
            vertex2 = list_vertices[1]

            sign = -1 if a > 0 else 1
            m_perpendicular = np.array([1, b / a]) * sign
            theta = np.arctan2(m_perpendicular[1], m_perpendicular[0])

            for vertex in [vertex2, vertex1]:
                # dist_diagonal * 100 is just an arbitrarily large distance
                x_new = vertex[0] + dist_diagonal * 100 * np.cos(theta)
                y_new = vertex[1] + dist_diagonal * 100 * np.sin(theta)
                vertex_new = [x_new, y_new]

                list_vertices.append(vertex_new)

        return Polygon(list_vertices)

