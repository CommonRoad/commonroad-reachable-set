#include "reachset/data_structure/reach/reach_polygon_boost.hpp"
#include "reachset/data_structure/reach/reach_vertex.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace reach;

ReachPolygon::ReachPolygon(vector<tuple<double, double>> const& vec_vertices) {
    if (vec_vertices.size() < 3)
        throw std::invalid_argument("<ReachPolygon> A polygon requires at least 3 vertices.");

    // prepare a vector of Boost.Geometry points
    vector<GeometryPoint> vec_points_geometry;
    vec_points_geometry.reserve(vec_vertices.size());
    for (auto const& vertex: vec_vertices) {
        vec_points_geometry.emplace_back(GeometryPoint{std::get<0>(vertex), std::get<1>(vertex)});
    }

    // create Boost.Geometry polygon
    _polygon = make_shared<GeometryPolygon>();
    _polygon->outer().assign(vec_points_geometry.begin(), vec_points_geometry.end());
    bg::correct(*_polygon);

    // update bounding box of the polygon
    update_bounding_box();
}

ReachPolygon::ReachPolygon(GeometryPolygon const& polygon, bool const& correct) {
    _polygon = make_shared<GeometryPolygon>();
    _polygon->outer().assign(polygon.outer().cbegin(), polygon.outer().cend());
    if (correct) bg::correct(*_polygon);

    // update bounding box of the polygon
    update_bounding_box();
}


ReachPolygonPtr ReachPolygon::clone() const {
    return make_shared<ReachPolygon>(*_polygon, false);
}


void ReachPolygon::update_bounding_box() {
    auto envelope = bg::return_envelope<GeometryBox>(*(_polygon));
    _box = {envelope.min_corner().x(), envelope.min_corner().y(),
            envelope.max_corner().x(), envelope.max_corner().y()};
}

ReachPolygonPtr ReachPolygon::from_rectangle_coordinates(double const& p_lon_min, double const& p_lat_min,
                                                         double const& p_lon_max, double const& p_lat_max) {
    vector<tuple<double, double>> vec_tuples{{p_lon_min, p_lat_min},
                                             {p_lon_max, p_lat_min},
                                             {p_lon_max, p_lat_max},
                                             {p_lon_min, p_lat_max}};

    return make_shared<ReachPolygon>(vec_tuples);
}

void ReachPolygon::intersect_halfspace(double const& a, double const& b, double const& c) {
    if (a == 0 and b == 0)
        throw std::invalid_argument("<ReachPolygon> Halfspace parameters not valid.");

    if (_polygon == nullptr) return;

    // construct halfspace polygon
    ReachPolygonPtr polygon_halfspace = construct_halfspace_polygon(a, b, c, _box);

    // take intersection
    vector<GeometryPolygon> vec_polygons_intersected;
    bg::intersection(*(_polygon), *(polygon_halfspace->_polygon), vec_polygons_intersected);

    if (not vec_polygons_intersected.empty()) {
        if (vec_polygons_intersected.size() != 1)
            cout << "<ReachPolygon> Intersection created multiple polygons.";

        _polygon = make_shared<GeometryPolygon>(vec_polygons_intersected[0]);

        // update bounding box of the polygon
        update_bounding_box();
    } else {
        _polygon = nullptr;
        _box = {{},
                {},
                {},
                {}};
    }
}


ReachPolygonPtr ReachPolygon::construct_halfspace_polygon(double const& a, double const& b, double const& c,
                                                          tuple<double, double, double, double> const& bounding_box) {
    // margin to enlarge the polygon at the boundaries
    auto margin = 10;
    auto[x_min, y_min, x_max, y_max] = bounding_box;
    auto dist_diagonal = std::sqrt(std::pow(x_max - x_min, 2) + std::pow(y_max - y_min, 2));

    vector<tuple<double, double>> vec_vertices;
    if (b == 0) { // horizontal
        if (a > 0) { // ax <= c
            vec_vertices.emplace_back(make_tuple(c / a, y_min - margin));
            vec_vertices.emplace_back(make_tuple(c / a, y_max + margin));
            vec_vertices.emplace_back(make_tuple(x_min - margin, y_max + margin));
            vec_vertices.emplace_back(make_tuple(x_min - margin, y_min - margin));
        } else { // -ax <= c
            vec_vertices.emplace_back(make_tuple(c / a, y_min - margin));
            vec_vertices.emplace_back(make_tuple(c / a, y_max + margin));
            vec_vertices.emplace_back(make_tuple(x_max + margin, y_max + margin));
            vec_vertices.emplace_back(make_tuple(x_max + margin, y_min - margin));
        }
    } else if (a == 0) { // vertical
        if (b > 0) { // by <= c
            vec_vertices.emplace_back(make_tuple(x_min - margin, c / b));
            vec_vertices.emplace_back(make_tuple(x_max + margin, c / b));
            vec_vertices.emplace_back(make_tuple(x_max + margin, y_min - margin));
            vec_vertices.emplace_back(make_tuple(x_min - margin, y_min - margin));
        } else { // -by <= c
            vec_vertices.emplace_back(make_tuple(x_min - margin, c / b));
            vec_vertices.emplace_back(make_tuple(x_max + margin, c / b));
            vec_vertices.emplace_back(make_tuple(x_max + margin, y_max + margin));
            vec_vertices.emplace_back(make_tuple(x_min - margin, y_max + margin));
        }
    } else { // general case
        // First compute two arbitrary vertices that are far away from the x coordinates, then compute the slope of the
        // vector that is perpendicular to the vector connecting these two vertices to look for remaining two vertices
        // necessary for the polygon construction.
        margin = 100;
        for (auto const& x: {x_min - margin, x_max + margin}) {
            double y = (-a * x + c) / b;
            vec_vertices.emplace_back(make_tuple(x, y));
        }
        auto vertex1 = vec_vertices[0];
        auto vertex2 = vec_vertices[1];

        auto sign = a > 0 ? -1 : 1;
        auto slope_perpendicular = make_tuple(1 * sign, b / a * sign);
        auto theta = std::atan2(std::get<1>(slope_perpendicular), std::get<0>(slope_perpendicular));

        for (auto const& vertex: {vertex2, vertex1}) {
            // dist_diagonal * 100 is just an arbitrarily large distance
            auto x_new = std::get<0>(vertex) + dist_diagonal * 100 * std::cos(theta);
            auto y_new = std::get<1>(vertex) + dist_diagonal * 100 * std::sin(theta);
            vec_vertices.emplace_back(make_tuple(x_new, y_new));
        }
    }

    return make_shared<ReachPolygon>(vec_vertices);
}

void ReachPolygon::convexify() {
    GeometryPolygon polygon_convexified;

    bg::convex_hull(*(_polygon), polygon_convexified);
    bg::assign(_polygon->outer(), polygon_convexified.outer());

    update_bounding_box();
}

void ReachPolygon::linear_mapping(double const& a11, double const& a12, double const& a21, double const& a22) {
    for (auto& vertex: _polygon->outer()) {
        auto x_new = a11 * vertex.x() + a12 * vertex.y();
        auto y_new = a21 * vertex.x() + a22 * vertex.y();
        vertex.set<0>(x_new);
        vertex.set<1>(y_new);
    }
    bg::correct(*(_polygon));

    update_bounding_box();
}


void ReachPolygon::print_vertices() const {
    for (auto const& vertex: vertices()) {
        cout << "\t(" << bg::get<0>(vertex) << ", " << bg::get<1>(vertex) << "), " << endl;
    }
}

void ReachPolygon::minkowski_sum(ReachPolygonPtr const& polygon_other) {
    if (this->empty() or polygon_other->empty()) return;

    auto vertices_self = this->vertices();
    auto vertices_other = polygon_other->vertices();
    auto const num_vertices_self = vertices_self.size();
    auto const num_vertices_other = vertices_other.size();
    bool reach_end_self = false, reach_end_other = false;
    auto idx_self = 0, idx_other = 0;

    Vertex vector_self, vector_other;
    vector<tuple<double, double>> vec_vertices_sum;
    while (true) {
        // insert summed vertex
        auto x_sum = vertices_self.at(idx_self % num_vertices_self).x() +
                     vertices_other.at(idx_other % num_vertices_other).x();
        auto y_sum = vertices_self.at(idx_self % num_vertices_self).y() +
                     vertices_other.at(idx_other % num_vertices_other).y();
        vec_vertices_sum.emplace_back(make_tuple(x_sum, y_sum));

        // shift to next vertex
        vector_self.x = vertices_self.at((idx_self + 1) % num_vertices_self).x() -
                        vertices_self.at(idx_self % num_vertices_self).x();
        vector_self.y = vertices_self.at((idx_self + 1) % num_vertices_self).y() -
                        vertices_self.at(idx_self % num_vertices_self).y();
        vector_other.x = vertices_other.at((idx_other + 1) % num_vertices_other).x() -
                         vertices_other.at(idx_other % num_vertices_other).x();
        vector_other.y = vertices_other.at((idx_other + 1) % num_vertices_other).y() -
                         vertices_other.at(idx_other % num_vertices_other).y();

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

    // remove duplicated vertices
    auto it_end = std::unique(vec_vertices_sum.begin(), vec_vertices_sum.end(),
                              [](tuple<double, double> const& a, tuple<double, double> const& b) {
                                  return std::get<0>(a) == std::get<0>(b) and
                                         std::get<1>(a) == std::get<1>(b);
                              });
    vec_vertices_sum.erase(it_end, vec_vertices_sum.end());

    if (vec_vertices_sum.size() > 1 and
        std::get<0>(*(vec_vertices_sum.end())) == std::get<0>(*(vec_vertices_sum.cbegin())) and
        std::get<1>(*(vec_vertices_sum.end())) == std::get<1>(*(vec_vertices_sum.cbegin()))) {
        vec_vertices_sum.erase(vec_vertices_sum.begin());
    }

    auto polygon_sum = ReachPolygon(vec_vertices_sum);
    bg::correct(*(polygon_sum.geometry_polygon()));
    polygon_sum.convexify();
    bg::assign(_polygon->outer(), polygon_sum.geometry_polygon()->outer());
    update_bounding_box();
}

bool ReachPolygon::intersects(ReachPolygonPtr const& polygon_other) const {
    return bg::intersects(*_polygon, *(polygon_other->geometry_polygon()));
}

vector<ReachPolygonPtr> ReachPolygon::intersection(ReachPolygonPtr const& polygon_other) const {
    std::deque<GeometryPolygon> intersection;
    vector<ReachPolygonPtr> result;

    bg::intersection(*_polygon, *(polygon_other->geometry_polygon()), intersection);

    result.reserve(intersection.size());
    for (auto const& polygon: intersection) {
        result.emplace_back(make_shared<ReachPolygon>(polygon));
    }

    return result;
}