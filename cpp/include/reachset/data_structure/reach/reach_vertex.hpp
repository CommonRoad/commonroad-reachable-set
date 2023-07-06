#pragma once

#include "reachset/utility/shared_include.hpp"

namespace reach {
/// Struct representing a vertex.
struct Vertex {
    double x{};
    double y{};

    Vertex() = default;

    Vertex(double const& x, double const& y) : x(x), y(y) {};

    inline double p_lon() const { return x; }

    inline double p_lat() const { return y; }

    Vertex operator-(Vertex const& other) const { return Vertex{x - other.x, y - other.y}; }

    Vertex operator+(Vertex const& other) const { return Vertex{x + other.x, y + other.y}; }

    bool operator==(Vertex const& other) const { return x == other.x && y == other.y; }

    /// Compares the angles of two vertices.
    static bool compare_angle(Vertex const& vertex1, Vertex const& vertex2);
};
}