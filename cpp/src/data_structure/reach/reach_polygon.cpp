#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace reach;

ReachPolygon::ReachPolygon() {
    _sorting_state = unsorted;
}

ReachPolygon::ReachPolygon(vector<Vertex> const& vec_vertices) {
    if (vec_vertices.size() < 3)
        throw std::invalid_argument("A polygon requires at least 3 vertices.");

    for (auto const& vertex: vec_vertices) {
        this->vec_vertices.emplace_back(vertex);
    }
    compute_bounding_box();
    _sorting_state = unsorted;
}

ReachPolygon::ReachPolygon(vector<tuple<double, double>> const& vec_vertices) {
    if (vec_vertices.size() < 3)
        throw std::invalid_argument("A polygon requires at least 3 vertices.");

    for (auto const& vertex: vec_vertices) {
        this->vec_vertices.emplace_back(Vertex{std::get<0>(vertex), std::get<1>(vertex)});
    }
    compute_bounding_box();
    _sorting_state = unsorted;
}

ReachPolygon::ReachPolygon(double const& p_lon_min, double const& p_lat_min,
                           double const& p_lon_max, double const& p_lat_max) {
    vector<Vertex> vec_vertices_temp{Vertex{p_lon_min, p_lat_min},
                                     Vertex{p_lon_max, p_lat_min},
                                     Vertex{p_lon_max, p_lat_max},
                                     Vertex{p_lon_min, p_lat_max}};
    for (auto const& vertex: vec_vertices_temp) {
        this->vec_vertices.emplace_back(vertex);
    }
    compute_bounding_box();
    _sorting_state = ccw;
}

ReachPolygon::ReachPolygon(std::tuple<double, double, double, double> const& tuple_coordinates) {
    auto[p_lon_min, p_lat_min, p_lon_max, p_lat_max] = tuple_coordinates;
    vector<Vertex> vec_vertices_temp{Vertex{p_lon_min, p_lat_min},
                                     Vertex{p_lon_max, p_lat_min},
                                     Vertex{p_lon_max, p_lat_max},
                                     Vertex{p_lon_min, p_lat_max}};
    for (auto const& vertex: vec_vertices_temp) {
        this->vec_vertices.emplace_back(vertex);
    }
    compute_bounding_box();
    _sorting_state = ccw;
}

ReachPolygon::ReachPolygon(vector<ReachPolygonPtr> const& vec_polygons) {
    auto num_vertices = vec_vertices.size();
    for (auto const& polygon: vec_polygons) {
        num_vertices += polygon->vec_vertices.size();
    }
    vec_vertices.reserve(num_vertices);

    for (auto const& polygon: vec_polygons) {
        for (auto const& vertex: polygon->vec_vertices) {
            vec_vertices.emplace_back(vertex);
        }
    }

    if (vec_vertices.size() < 3)
        throw std::invalid_argument("A polygon requires at least 3 vertices.");

    compute_bounding_box();
    _sorting_state = unsorted;
}

void ReachPolygon::_sort_vertices_left_to_right() {
    if (_sorting_state == left_to_right) { return; }

    std::sort(vec_vertices.begin(), vec_vertices.end(),
              [](Vertex const& vertex1, Vertex const& vertex2) {
                  if (vertex1.x < vertex2.x) { return true; }
                  if (vertex1.x > vertex2.x) { return false; }
                  if (vertex1.y < vertex2.y) { return true; }
                  if (vertex1.y > vertex2.y) { return false; }
                  return false;
              });

    _sorting_state = left_to_right;
}

/// Removes an element if it is equal to its predecessor is equal. The first element is compared with the last
/// element. The meaning of the operation depends on how the vertex array is sorted.
void ReachPolygon::_remove_duplicated_vertices() {
    // obtain unique vertices
    auto idx_last = std::unique(vec_vertices.begin(), vec_vertices.end(),
                                [](Vertex const& vertex1, Vertex const& vertex2) {
                                    return vertex1.x == vertex2.x && vertex1.y == vertex2.y;
                                });
    vec_vertices.erase(idx_last, vec_vertices.end());

    if (vec_vertices.size() > 1 &&
        vec_vertices[vec_vertices.size() - 1].x == vec_vertices[0].x &&
        vec_vertices[vec_vertices.size() - 1].y == vec_vertices[0].y) {
        vec_vertices.erase(vec_vertices.begin());
    }
}

void ReachPolygon::print_info() const {
    auto num_vertices = vec_vertices.size();
    cout << "==============" << endl;
    cout << "# of vertices: " << num_vertices << endl;
    cout << "bounding box: " << p_lon_min() << ", " << p_lat_min() << ", " << p_lon_max() << ", " << p_lat_max()
         << endl;

    for (auto const& vec: vec_vertices) {
        cout << "(" << vec.x << ", " << vec.y << ")" << endl;
    }
}

void ReachPolygon::compute_bounding_box() {
    auto p_lon_min = std::numeric_limits<double>::infinity();
    auto p_lat_min = std::numeric_limits<double>::infinity();
    auto p_lon_max = -std::numeric_limits<double>::infinity();
    auto p_lat_max = -std::numeric_limits<double>::infinity();

    for (auto const& vertex: vec_vertices) {
        p_lon_min = std::min(p_lon_min, vertex.p_lon());
        p_lat_min = std::min(p_lat_min, vertex.p_lat());
        p_lon_max = std::max(p_lon_max, vertex.p_lon());
        p_lat_max = std::max(p_lat_max, vertex.p_lat());
    }

    _box = {p_lon_min, p_lat_min, p_lon_max, p_lat_max};
}


void ReachPolygon::sort_vertices_bottom_left_first() {
    if (_sorting_state == ccw_bottom_left_first) { return; }
    convexify();

    // find bottom left element
    int min_i = 0;
    for (int i = 1; i < vec_vertices.size(); i++) {
        if (vec_vertices[min_i].y > vec_vertices[i].y) {
            min_i = i;
        } else if (vec_vertices[min_i].y == vec_vertices[i].y) {
            if (vec_vertices[min_i].x > vec_vertices[i].x) {
                min_i = i;
            }
        }
    }

    // swap all elements
    vector<Vertex> vertices_swap;
    vertices_swap.resize(vec_vertices.size());
    for (int i = 0; i < vec_vertices.size(); i++) {
        vertices_swap[i] = get_vertex_with_cyclic_index(min_i + i);
    }

    vec_vertices = vertices_swap;
    _sorting_state = ccw_bottom_left_first;
}

void ReachPolygon::convexify() {
    if (_sorting_state == ccw || _sorting_state == ccw_bottom_left_first) return;

    _sort_vertices_left_to_right();
    _remove_duplicated_vertices();

    if (vec_vertices.size() < 3) {
        _sorting_state = ccw;
        return;
    }

    auto cross = [](Vertex const& o, Vertex const& a, Vertex const& b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };

    int k = 0;
    vector<Vertex> hull(2 * vec_vertices.size());

    // Build lower hull
    for (auto& vertex: vec_vertices) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], vertex) <= 0) {
            k--;
        }
        hull[k++] = vertex;
    }

    // Build upper hull
    for (int i = static_cast<int>(vec_vertices.size()) - 2, t = k + 1; i >= 0; i--) {
        while (k >= t && cross(hull[k - 2], hull[k - 1], vec_vertices[i]) <= 0) {
            k--;
        }
        hull[k++] = vec_vertices[i];
    }

    hull.resize(k - 1); // modified from original version
    vec_vertices = hull;
    _sorting_state = ccw;
    compute_bounding_box();
}

/// Searches in counter-clockwise order through vertices for two vertices marked by index r and q:
/// vertex[q] is inside the intersected polyhedron, but its previous vertex not
/// vertex[r] is inside the intersected polyhedron, but its successor vertex not
/// If the intersected polyhedron is empty or equals the original one, then there is no valid q and r.
/// The new polygon is constructed using the vertices[r..q] plus the two intersections point with the halfspace.
void ReachPolygon::intersect_halfspace(double a, double b, double c) {
    if (a == 0 and b == 0) {
        cout << "Halfspace parameters are invalid." << endl;
        return;
    }
    if (vec_vertices.empty()) { return; }

    bool any_inside = false;

    auto inside = [&a, &b, &c, &any_inside, this](std::size_t i) {
        bool ret = (a * vec_vertices[i].x + b * vec_vertices[i].y <= c);
        if (ret) { any_inside = true; }
        return ret;
    };

    // intersection between a line in parametric representation and another line
    // given by two points
    auto intersection = [&a, &b, &c](Vertex& pt1, Vertex& pt2) {
        double a21 = pt2.y - pt1.y;
        double a22 = -(pt2.x - pt1.x);
        double b2 = a21 * pt2.x + a22 * pt2.y;
        double detA = a * a22 - b * a21;

        return Vertex{(c * a22 - b2 * b) / detA, (a * b2 - a21 * c) / detA};
    };

    if (vec_vertices.size() == 1) {
        if (inside(0)) return;
        else {
            vec_vertices.clear();
            _sorting_state = ccw;
            return;
        }
    }

    int q = -1; // q is inside, prev(q) is not inside
    int r = -1; // r is inside, next(r) is not inside

    for (int i = 1; i < vec_vertices.size() - 1; i++) {
        if (inside(i)) {
            if (!inside(i - 1)) { q = i; }
            if (!inside(i + 1)) { r = i; }
        }
    }
    if (!inside(vec_vertices.size() - 1) && inside(0)) { q = 0; }
    if (!inside(vec_vertices.size() - 2) && inside(vec_vertices.size() - 1)) {
        q = static_cast<int>(vec_vertices.size()) - 1;
    }

    if (inside(vec_vertices.size() - 1) && !inside(0)) {
        r = static_cast<int>(vec_vertices.size()) - 1;
    }
    if (inside(0) && !inside(1)) { r = 0; }

    assert(q == -1 ? r == q : true); // if q==-1 then also r==-1
    if (q == -1) {
        if (any_inside) return;
        else {
            // polyhedron is empty
            vec_vertices.clear();
            _sorting_state = ccw;
            return;
        }
    }

    int qq = q - 1 < 0 ? static_cast<int>(vec_vertices.size()) - 1 : q - 1; // qq = prev(q)
    int rr = r + 1 > vec_vertices.size() - 1 ? 0 : r + 1; // rr = next(rr)

    // create new polygon by taking i elements from old and add two intersection
    // points.
    // This is not correct if the polyhedron has only two vertices, but
    // "removeDuplicateVec2" corrects this.
    size_t n = (r >= q ? r - q + 1 : r + vec_vertices.size() - q + 1);
    vector<Vertex> vertices_new;
    vertices_new.reserve(n + 2);
    for (size_t i = 0; i < n; i++) {
        vertices_new.push_back(get_vertex_with_cyclic_index(q + i));
    }

    // add intersection points with halfspace
    vertices_new.emplace_back(intersection(vec_vertices[r], vec_vertices[rr]));
    vertices_new.emplace_back(intersection(vec_vertices[q], vec_vertices[qq]));

    vec_vertices = vertices_new;
    _sorting_state = ccw;
    _remove_duplicated_vertices();
    compute_bounding_box();
}


void ReachPolygon::minkowski_sum(std::shared_ptr<ReachPolygon> const& polygon_other) {
    if (this->empty() or polygon_other->empty()) return;

    auto polygon_sum = make_shared<ReachPolygon>();
    vector<tuple<double, double>> vec_vertices_sum;
    // prepare for summation by convexify and sorting the vertices of the polygons
    sort_vertices_bottom_left_first();

    auto vertices_self = this->vertices();
    auto vertices_other = polygon_other->vertices();
    auto const num_vertices_self = vertices_self.size();
    auto const num_vertices_other = vertices_other.size();
    bool reach_end_self = false, reach_end_other = false;
    auto idx_self = 0, idx_other = 0;

    Vertex vector_self, vector_other;
    while (true) {
        polygon_sum->add_vertex(get_vertex_with_cyclic_index(idx_self) +
                                polygon_other->get_vertex_with_cyclic_index(idx_other));
        vector_self = get_vertex_with_cyclic_index(idx_self + 1) -
                      get_vertex_with_cyclic_index(idx_self);
        vector_other = polygon_other->get_vertex_with_cyclic_index(idx_other + 1) -
                       polygon_other->get_vertex_with_cyclic_index(idx_other);

        if (reach_end_self)
            idx_other++;

        else if (reach_end_other)
            idx_self++;

        else {
            if (Vertex::compare_angle(vector_self, vector_other))
                idx_self++;

            else if (Vertex::compare_angle(vector_other, vector_self))
                idx_other++;

            else {
                idx_self++;
                idx_other++;
            }
        }

        if (idx_self >= num_vertices_self) {
            reach_end_self = true;
            idx_self = 0;
        }

        if (idx_other >= num_vertices_other) {
            reach_end_other = true;
            idx_other = 0;
        }

        if (reach_end_self and reach_end_other) {
            break;
        }

    }

    polygon_sum->_remove_duplicated_vertices();

    vec_vertices.clear();
    vec_vertices = polygon_sum->vertices();
    _sorting_state = ccw_bottom_left_first;
    compute_bounding_box();
}