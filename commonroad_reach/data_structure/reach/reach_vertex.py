class Vertex:
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

    @staticmethod
    def compare_angle(vertex1: 'Vertex', vertex2: 'Vertex'):
        c1 = vertex1.x * vertex1.x / (vertex1.x * vertex1.x + vertex1.y * vertex1.y)
        c2 = vertex2.x * vertex2.x / (vertex2.x * vertex2.x + vertex2.y * vertex2.y)

        if vertex1.x < 0:
            sign1 = 1
        else:
            sign1 = -1

        if vertex1.y < 0:
            add1 = 3
            sign1 = sign1 * -1
        else:
            add1 = 1
        if vertex2.x < 0:
            sign2 = 1
        else:
            sign2 = -1

        if vertex2.y < 0:
            add2 = 3
            sign2 = sign2 * -1
        else:
            add2 = 1

        c1 = add1 + sign1 * c1
        c2 = add2 + sign2 * c2

        return c1 < c2
