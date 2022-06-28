class Vertex:
    """
    Class to represent a vertex of the polygon.
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        return Vertex(x, y)

    def __sub__(self, other):
        x = self.x - other.x
        y = self.y - other.y
        return Vertex(x, y)

    def __eq__(self, other):
        if not isinstance(other, Vertex):
            return False

        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    @property
    def p_lon(self):
        return self.x

    @property
    def p_lat(self):
        return self.y
