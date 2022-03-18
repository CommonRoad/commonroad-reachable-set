#include "reachset/data_structure/reach/reach_polygon2.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace reach;

ReachPolygon2::ReachPolygon2() {
    _sorting_state = unsorted;
}

ReachPolygon2::ReachPolygon2(vector<Vertex> const& vec_vertices){
    for (auto const& vertex: vec_vertices) {
        this->vec_vertices.emplace_back(vertex);
    }
    _sorting_state = unsorted;
}

ReachPolygon2::ReachPolygon2(vector<ReachPolygon2Ptr> const& vec_polygons) {
    // todo: replace by k-way merge
    auto num_vertices = vec_vertices.size();
    for (auto const& polygon: vec_polygons) {
        num_vertices += polygon->vec_vertices.size();
    }
    vec_vertices.reserve(num_vertices);

    for (auto const& polygon: vec_polygons) {
        for (auto const& vertex: polygon->vec_vertices) {
            vec_vertices.push_back(vertex);
        }
    }
    _sorting_state = unsorted;
}

void ReachPolygon2::print_info() const {
    auto num_vertices = vec_vertices.size();
    cout << "==============" << endl;
    cout << "# of vertices: " << num_vertices << endl;
    cout << "bounding box: " << p_lon_min() << ", " << p_lat_min() << ", " << p_lon_max() << ", " << p_lat_max() << endl;

    for (auto const& vec: vec_vertices) {
        cout << "(" << vec.x << ", " << vec.y << ")" << endl;
    }
}

void ReachPolygon2::update_bounding_box() {
    assert(vec_vertices.size() > 0);

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

void ReachPolygon2::_sort_vertices_left_to_right() {
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

void ReachPolygon2::_sort_vertices_bottom_left_first() {
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
            } else if (vec_vertices[min_i].x == vec_vertices[i].x) {
                std::cout << i << " " << vec_vertices[min_i].x << " " << vec_vertices[i].x << std::endl;
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

/// Removes an element if it is equal to its predecessor is equal. The first element is compared with the last
/// element. The meaning of the operation depends on how the vertex array is sorted.
void ReachPolygon2::_remove_duplicated_vertices() {
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

void ReachPolygon2::convexify() {
    if (_sorting_state == ccw || _sorting_state == ccw_bottom_left_first) {
        return;
    }

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

    // Build the lower hull
    for (auto const& vertex: vec_vertices) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], vertex) <= 0) {
            k--;
        }
        hull[k++] = vertex;
    }

    // Build the upper hull
    for (size_t i = vec_vertices.size() - 2, t = k + 1; i >= 0; i--) {
        while (k >= t && cross(hull[k - 2], hull[k - 1], vec_vertices[i]) <= 0) {
            k--;
        }
        hull[k++] = vec_vertices[i];
    }

    hull.resize(k - 1); // modified from original version
    vec_vertices = hull;
    _sorting_state = ccw;
}

/// Searches in counter-clockwise order through vertices for two vertices marked by index r and q:
/// vertex[q] is inside the intersected polyhedron, but its previous vertex not
/// vertex[r] is inside the intersected polyhedron, but its successor vertex not
/// If the intersected polyhedron is empty or equals the original one, then there is no valid q and r.
/// The new polygon is constructed using the vertices[r..q] plus the two intersections point with the halfspace.
void ReachPolygon2::intersect_half_space(double a1, double a2, double b) {
    convexify();
    if (vec_vertices.empty()) { return; }

    bool any_inside = false;
    auto inside = [&a1, &a2, &b, &any_inside, this](size_t i) {
        bool ret = (a1 * vec_vertices[i].x + a2 * vec_vertices[i].y < b);
        if (ret) { any_inside = true; }
        return ret;
    };

    // intersection between a line in parametric representation and another line given by two points
    auto intersection = [&a1, &a2, &b](Vertex& pt1, Vertex& pt2) {
        double a21 = pt2.y - pt1.y;
        double a22 = -(pt2.x - pt1.x);
        double b2 = a21 * pt2.x + a22 * pt2.y;
        double detA = a1 * a22 - a2 * a21;

        return Vertex{(b * a22 - b2 * a2) / detA, (a1 * b2 - a21 * b) / detA};
    };

    if (vec_vertices.size() == 1) {
        if (inside(0)) {
            return;

        } else {
            vec_vertices.clear();
            _sorting_state = ccw;
            return;
        }
    }

    size_t q = -1; // q is inside, prev(q) is not inside
    size_t r = -1; // r is inside, next(r) is not inside

    for (int i = 1; i < vec_vertices.size() - 1; i++) {
        if (inside(i)) {
            if (!inside(i - 1)) { q = i; }
            if (!inside(i + 1)) { r = i; }
        }
    }
    if (!inside(vec_vertices.size() - 1) && inside(0)) { q = 0; }
    if (!inside(vec_vertices.size() - 2) && inside(vec_vertices.size() - 1)) { q = vec_vertices.size() - 1; }

    if (inside(vec_vertices.size() - 1) && !inside(0)) { r = vec_vertices.size() - 1; }
    if (inside(0) && !inside(1)) { r = 0; }

    assert(q == -1 ? r == q : true); // if q==-1 then also r==-1
    if (q == -1) {
        if (any_inside) {
            return;

        } else {
            // polygon is empty
            vec_vertices.clear();
            _sorting_state = ccw;
            return;
        }
    }

    size_t qq = q - 1 < 0 ? vec_vertices.size() - 1 : q - 1; // qq = prev(q)
    size_t rr = r + 1 > vec_vertices.size() - 1 ? 0 : r + 1; // rr = next(rr)

    // create new polygon by taking i elements from old and add two intersection points.
    // This is not correct if the polyhedron has only two vertices, but "_remove_duplicated_vertices" corrects this.
    std::size_t n = (r >= q ? r - q + 1 : r + vec_vertices.size() - q + 1);
    std::vector<Vertex> vertices_new;
    vertices_new.reserve(n + 2);
    for (size_t i = 0; i < n; i++) {
        vertices_new.push_back(get_vertex_with_cyclic_index(q + i));
    }

    vertices_new.emplace_back(intersection(vec_vertices[r], vec_vertices[rr]));
    vertices_new.emplace_back(intersection(vec_vertices[q], vec_vertices[qq]));

    vec_vertices = vertices_new;
    _sorting_state = ccw;
    _remove_duplicated_vertices();
}



//ReachPolygon2::ReachPolygon2(vector<tuple<double, double>> const& vec_vertices) {
//    if (vec_vertices.size() < 3)
//        throw std::invalid_argument("<ReachPolygon> A polygon requires at least 3 vertices.");
//
//    // prepare a vector of Boost.Geometry points
//    vector<GeometryPoint> vec_points_geometry;
//    vec_points_geometry.reserve(vec_vertices.size());
//    for (auto const& vertex: vec_vertices) {
//        vec_points_geometry.emplace_back(GeometryPoint{std::get<0>(vertex), std::get<1>(vertex)});
//    }
//
//    // create Boost.Geometry polygon
//    _polygon = make_shared<GeometryPolygon>();
//    _polygon->outer().assign(vec_points_geometry.begin(), vec_points_geometry.end());
//    bg::correct(*_polygon);
//
//    // update bounding box of the polygon
//    update_bounding_box();
//}
//
//ReachPolygon2::ReachPolygon2(GeometryPolygon const& polygon, bool const& correct) {
//    _polygon = make_shared<GeometryPolygon>();
//    _polygon->outer().assign(polygon.outer().cbegin(), polygon.outer().cend());
//    if (correct) bg::correct(*_polygon);
//
//    // update bounding box of the polygon
//    update_bounding_box();
//}