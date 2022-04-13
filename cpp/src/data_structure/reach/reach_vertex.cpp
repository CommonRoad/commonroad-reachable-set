#include "reachset/data_structure/reach/reach_vertex.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace reach;

bool Vertex::compare_angle(Vertex const& vertex1, Vertex const& vertex2) {
    int sign1, sign2;
    int add1, add2;
    double c1 = vertex1.x * vertex1.x / (vertex1.x * vertex1.x + vertex1.y * vertex1.y);
    double c2 = vertex2.x * vertex2.x / (vertex2.x * vertex2.x + vertex2.y * vertex2.y);

    if (vertex1.x < 0) {
        sign1 = 1;

    } else {
        sign1 = -1;
    }

    if (vertex1.y < 0) {
        add1 = 3;
        sign1 = sign1 * -1;

    } else {
        add1 = 1;
    }

    if (vertex2.x < 0) {
        sign2 = 1;

    } else {
        sign2 = -1;
    }

    if (vertex2.y < 0) {
        add2 = 3;
        sign2 = sign2 * -1;

    } else {
        add2 = 1;
    }

    c1 = add1 + sign1 * c1;
    c2 = add2 + sign2 * c2;

    return c1 < c2;
}