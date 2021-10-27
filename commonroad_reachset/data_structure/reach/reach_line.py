from shapely.geometry import LineString


class ReachLine(LineString):
    """LineString class used in geometric operation."""

    def __init__(self, p_lon_min, p_lat_min, p_lon_max, p_lat_max):
        self.p_lon_min = p_lon_min
        self.p_lat_min = p_lat_min
        self.p_lon_max = p_lon_max
        self.p_lat_max = p_lat_max

        super().__init__([(p_lon_min, p_lat_min), (p_lon_max, p_lat_max)])

    def __repr__(self):
        return f"ReachLine({self.p_lon_min},{self.p_lon_max},{self.p_lat_min},{self.p_lat_max})"
