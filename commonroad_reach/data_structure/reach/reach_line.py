from typing import Tuple

from shapely.geometry import LineString


class ReachLine:
    """
    Line segment used in geometric operations.

    A line segment is formed with two vertices.
    """

    def __init__(self, p_lon_min: float, p_lat_min: float, p_lon_max: float, p_lat_max: float):
        self.p_lon_min = p_lon_min
        self.p_lat_min = p_lat_min
        self.p_lon_max = p_lon_max
        self.p_lat_max = p_lat_max

        self._shapely_line_string = LineString([(p_lon_min, p_lat_min), (p_lon_max, p_lat_max)])

    def __repr__(self):
        return f"ReachLine({self.p_lon_min},{self.p_lon_max},{self.p_lat_min},{self.p_lat_max})"

    @property
    def shapely_object(self) -> LineString:
        return self._shapely_line_string

    @property
    def bounds(self) -> Tuple:
        return self._shapely_line_string.bounds
