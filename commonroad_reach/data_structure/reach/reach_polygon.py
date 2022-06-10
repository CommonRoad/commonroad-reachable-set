import logging
logger = logging.getLogger(__name__)

from abc import ABC
from typing import List, Tuple, Union

import numpy as np
from shapely.geometry import Polygon, MultiPolygon


class ReachPolygon(Polygon, ABC):
    """Polygon class that constitutes reachset nodes and position rectangles.

    When used to represent a reachset node, it is defined in the position-velocity domain, and can be used to
    represent a polygon in either the longitudinal or the lateral direction; When used to represent a position,
    it is defined in the longitudinal/lateral position domain.
    """

    def __init__(self, list_vertices: list, fix_vertices=True):
        if isinstance(list_vertices, list):
            if len(list_vertices) < 3:
                message = "A polygon needs at least 3 vertices."
                logger.error(message)
                raise Exception(message)

            # Shapely polygon requires identical initial and final vertices
            if fix_vertices and not np.allclose(list_vertices[0], list_vertices[-1]):
                list_vertices.append(list_vertices[0])

        super(ReachPolygon, self).__init__(list_vertices)

    def __repr__(self):
        return f"ReachPolygon({self.bounds[0]:.4}, {self.bounds[1]:.4}, {self.bounds[2]:.4}, {self.bounds[3]:.4})"

    def __str__(self):
        return f"{self.bounds}"

    @property
    def p_min(self):
        """Minimum position in the position-velocity domain."""
        return self.bounds[0]

    @property
    def p_max(self):
        """Maximum position in the position-velocity domain."""
        return self.bounds[2]

    @property
    def v_min(self):
        """Minimum velocity in the position-velocity domain."""
        return self.bounds[1]

    @property
    def v_max(self):
        """Maximum velocity in the position-velocity domain."""
        return self.bounds[3]

    @property
    def p_lon_min(self):
        """Minimum longitudinal position in the position domain."""
        return self.bounds[0]

    @property
    def p_lon_max(self):
        """Maximum longitudinal position in the position domain."""
        return self.bounds[2]

    @property
    def p_lon_center(self):
        """Center longitudinal position in the position domain."""
        return (self.p_lon_min + self.p_lon_max) / 2

    @property
    def p_lat_min(self):
        """Minimum lateral position in the position domain."""
        return self.bounds[1]

    @property
    def p_lat_max(self):
        """Maximum lateral position in the position domain."""
        return self.bounds[3]

    @property
    def p_lat_center(self):
        """Center lateral position in the position domain."""
        return (self.p_lat_min + self.p_lat_max) / 2

    @property
    def diagonal_squared(self):
        """Square length of the diagonal of the position domain."""
        return (self.p_lon_max - self.p_lon_min) ** 2 + (self.p_lat_max - self.p_lat_min) ** 2

    @classmethod
    def from_polygon(cls, polygon: Polygon):
        """Returns a polygon given another polygon."""
        if polygon.is_empty:
            return None

        else:
            return ReachPolygon(cls.get_vertices(polygon))

    @staticmethod
    def from_rectangle_vertices(p_lon_min, p_lat_min, p_lon_max, p_lat_max) -> "ReachPolygon":
        """Returns a polygon given the vertices of a rectangle."""
        list_vertices = [(p_lon_min, p_lat_min), (p_lon_max, p_lat_min),
                         (p_lon_max, p_lat_max), (p_lon_min, p_lat_max)]
        return ReachPolygon(list_vertices)

    def clone(self, convexify) -> "ReachPolygon":
        """Returns a cloned polygon."""
        if convexify:
            return ReachPolygon.from_polygon(self.convex_hull)

        else:
            return ReachPolygon(self.vertices)

    def intersect_halfspace(self, a, b, c) -> Union["ReachPolygon", None]:
        """Returns the intersection of the polygon and the halfspace specified in the form of ax + by <= c."""
        assert not (a == 0 and b == 0), "Halfspace parameters are not valid."

        polygon_halfspace = self.construct_halfspace_polygon(a, b, c, self.bounds)
        polygon_intersected = self.intersection(polygon_halfspace)

        if isinstance(polygon_intersected, Polygon) and not polygon_intersected.is_empty:
            return ReachPolygon.from_polygon(polygon_intersected)

        else:
            return None

    @property
    def vertices(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Returns the list of vertices of the polygon."""
        if isinstance(self, Polygon):
            list_x, list_y = self.exterior.coords.xy

        elif isinstance(self, MultiPolygon):
            list_x = []
            list_y = []
            for polygon in self:
                list_x.extend(polygon.exterior.coords.xy[0])
                list_y.extend(polygon.exterior.coords.xy[1])

        else:
            message = "Type error."
            logger.error(message)
            raise Exception(message)

        list_vertices = [vertex for vertex in zip(list_x, list_y)]

        return list_vertices[:-1]

    @staticmethod
    def get_vertices(polygon) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Returns the list of vertices of the polygon."""
        if isinstance(polygon, Polygon) or isinstance(polygon, ReachPolygon):
            list_x, list_y = polygon.exterior.coords.xy

        elif isinstance(polygon, MultiPolygon):
            list_x = []
            list_y = []
            for plg in polygon:
                list_x.extend(plg.exterior.coords.xy[0])
                list_y.extend(plg.exterior.coords.xy[1])

        else:
            message = "Type error."
            logger.error(message)
            raise Exception(message)

        list_vertices = [vertex for vertex in zip(list_x, list_y)]

        return list_vertices

    @staticmethod
    def construct_halfspace_polygon(a, b, c, bounds_polygon) -> Polygon:
        """Returns a polygon representing the halfspace."""
        x_min, y_min, x_max, y_max = bounds_polygon
        dist_diagonal = ((x_max - x_min) ** 2 + (y_max - y_min) ** 2) ** 0.5
        margin = 10

        list_vertices = []

        if b == 0:
            # vertical
            if a > 0:  # x <= c/a
                list_vertices.append([c/a, y_min - margin])
                list_vertices.append([c/a, y_max + margin])
                list_vertices.append([x_min - margin, y_max + margin])
                list_vertices.append([x_min - margin, y_min - margin])

            else:  # x >= c/a
                list_vertices.append([c/a, y_min - margin])
                list_vertices.append([c/a, y_max + margin])
                list_vertices.append([x_max + margin, y_max + margin])
                list_vertices.append([x_max + margin, y_min - margin])

        elif a == 0:
            # horizontal
            if b > 0:  # by <= c
                list_vertices.append([x_min - margin, c/b])
                list_vertices.append([x_max + margin, c/b])
                list_vertices.append([x_max + margin, y_min - margin])
                list_vertices.append([x_min - margin, y_min - margin])

            else:  # by <= c
                list_vertices.append([x_min - margin, c/b])
                list_vertices.append([x_max + margin, c/b])
                list_vertices.append([x_max + margin, y_max + margin])
                list_vertices.append([x_min - margin, y_max + margin])

        else:
            # general case
            """
            First compute two arbitrary vertices that are far away from the x boundary,
            then compute the slope of the vector that is perpendicular to the vector
            connecting these two points to look for remaining two vertices required
            for the polygon construction.
            """
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

        return ReachPolygon(list_vertices)
