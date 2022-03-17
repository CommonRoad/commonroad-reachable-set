#pragma once

#include "reachset/utility/geometry_definition.hpp"
#include "reachset/utility/shared_include.hpp"
#include "reachset/data_structure/reach/reach_vertex.hpp"

namespace reach {
/// Polygon class used in reachset nodes and position rectangles.
/// When used to represent a reachset node, it is defined in the position-velocity domain, and can be used to represent
/// a polygon in either the longitudinal or the lateral direction; When used to represent a position, it is defined in
/// the longitudinal/lateral position domain.
class ReachPolygon2 {
private:
    enum SortingState {
        unsorted = 0, left_to_right = 1, ccw = 10, ccw_bottom_left_first = 11
    };
    SortingState _sorting_state;

    std::tuple<double, double, double, double> _box;

    void _sort_vertices_left_to_right();

    void _sort_vertices_bottom_left_first();

    void _remove_duplicated_vertices();

    //GeometryPolygonPtr _polygon;

public:
    ReachPolygon2();

    explicit ReachPolygon2(std::vector<std::shared_ptr<ReachPolygon2>> const& vec_polygons);

    std::vector<Vertex> vec_vertices;

    inline std::tuple<double, double, double, double> bounding_box() const { return this->_box; }

    inline double p_min() const { return std::get<0>(this->_box); }

    inline double p_max() const { return std::get<2>(this->_box); }

    inline double v_min() const { return std::get<1>(this->_box); }

    inline double v_max() const { return std::get<3>(this->_box); }

    inline double p_lon_min() const { return std::get<0>(this->_box); }

    inline double p_lon_max() const { return std::get<2>(this->_box); }

    inline double p_lat_min() const { return std::get<1>(this->_box); }

    inline double p_lat_max() const { return std::get<3>(this->_box); }

    inline double p_lon_center() const { return (p_lon_min() + p_lon_max()) / 2.0; }

    inline double p_lat_center() const { return (p_lat_min() + p_lat_max()) / 2.0; }

    inline bool empty() const { return vec_vertices.empty(); }

    // todo: why need to convexify here?
    inline std::vector<Vertex> const& vertices() {
        convexify();
        return vec_vertices;
    }

    inline Vertex const& get_vertex_with_cyclic_index(size_t i) const {
        assert(i >= 0);
        return vec_vertices[i % vec_vertices.size()];
    }

    /// Updates the bounding box of the polygon.
    void update_bounding_box();

    inline void add_vertex(double x, double y) {
        vec_vertices.emplace_back(Vertex{x, y});
    }

    inline void add_vertex(Vertex v) {
        vec_vertices.emplace_back(v);
    }

    inline void add_polygon(std::shared_ptr<ReachPolygon2> const& polygon) {
        vec_vertices.insert(vec_vertices.end(), polygon->vec_vertices.begin(), polygon->vec_vertices.end());
        _sorting_state = unsorted;
    }

    /// Convexify the polygon.
    /// @note using Andrew monotone chain, see <a href="https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain">
    void convexify();

    inline void linear_map(double a11, double a12, double a21, double a22) {
        convexify();
        for (auto& vertex: vec_vertices) {
            double tmp = a11 * vertex.x + a12 * vertex.y;
            vertex.y = a21 * vertex.x + a22 * vertex.y;
            vertex.x = tmp;
        }
        _sorting_state = ccw;
        _remove_duplicated_vertices();
    }

    void intersect_half_space(double a1, double a2, double b);










    ///// Constructor of ReachPolygon.
    ///// @param vec_vertices vector of tuples representing the vertices
    //explicit ReachPolygon2(std::vector<std::tuple<double, double>> const& vec_vertices);
    //
    ////explicit ReachPolygon2(GeometryPolygon const& polygon, bool const& correct = true);
    //
    //
    //std::shared_ptr<ReachPolygon2> clone() const;
    //
    //inline GeometryPolygonPtr geometry_polygon() { return this->_polygon; }
    //
    //inline GeometryRing vertices() const { return this->_polygon->outer(); }
    //
    //inline bool valid() const { return _polygon != nullptr; };
    //
    //
    ///// Intersects with the halfspace specified in the form of ax + by <= c.
    //void intersect_halfspace(double const& a, double const& b, double const& c);
    //
    /////// Linear mapping in two dimensional plane.
    /////// Computes the zero-input response of the system.
    /////// @note see <a href="https://ieeexplore.ieee.org/abstract/document/8047450?casa_token=kfXwS_YEKoEAAAAA:k6TBp5W4m1BDxAZuMWI8S8T1I-fe38s3txJkzuNjiUMPv2T9Ccp1rzCmzNx8Z27VnnIgA4LU-Q">
    /////// *Computing the Drivable Area of Autonomous Road Vehicles in Dynamic Road Scenes*</a> , Sec.IV.A.
    ////void linear_mapping(double const& a11, double const& a12, double const& a21, double const& a22);
    //
    ///// Computes the minkowski sum with the input polygon.
    ///// @note Taken from Sebastian Söntges/Stefanie Manzinger's implementation.
    ///// 5x faster than naively computing all new vertices and taking convex hull with Boost.Geometry.
    //void minkowski_sum(std::shared_ptr<ReachPolygon2> const& polygon_other);
    //
    //void print_vertices() const;
    //
    //bool intersects(std::shared_ptr<ReachPolygon2> const& polygon_other) const;
    //
    //std::vector<std::shared_ptr<ReachPolygon2>> intersection(std::shared_ptr<ReachPolygon2> const& polygon_other) const;
    //
    ///// Creates a polygon from the given coordinates of a bounding box.
    ///// @param p_lon_min minimum longitudinal position
    ///// @param p_lat_min minimum lateral position
    ///// @param p_lon_max maximum longitudinal position
    ///// @param p_lat_max maximum lateral position
    //static std::shared_ptr<ReachPolygon2> from_rectangle_coordinates(double const& p_lon_min, double const& p_lat_min,
    //                                                                 double const& p_lon_max, double const& p_lat_max);
    //
    ///// Constructs a halfspace with the given parameters in the form of ax + by <= c.
    ///// A rectangle is constructed to represent the halfspace.
    //static std::shared_ptr<ReachPolygon2>
    //construct_halfspace_polygon(double const& a, double const& b, double const& c,
    //                            std::tuple<double, double, double, double> const& bounding_box);
};

using ReachPolygon2Ptr = std::shared_ptr<ReachPolygon2>;

}