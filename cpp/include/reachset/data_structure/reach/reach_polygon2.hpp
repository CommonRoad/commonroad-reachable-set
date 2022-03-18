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

    std::tuple<double, double, double, double> _box{std::numeric_limits<double>::infinity(),
                                                    std::numeric_limits<double>::infinity(),
                                                    -std::numeric_limits<double>::infinity(),
                                                    -std::numeric_limits<double>::infinity()};

    void _sort_vertices_left_to_right();

    void _remove_duplicated_vertices();

    //GeometryPolygonPtr _polygon;

public:
    ReachPolygon2();

    explicit ReachPolygon2(std::vector<Vertex> const& vec_vertices);

    explicit ReachPolygon2(std::vector<std::tuple<double,double>> const& vec_vertices);

    explicit ReachPolygon2(std::tuple<double, double, double, double> const& tuple_coordinates);

    ReachPolygon2(double const& p_lon_min, double const& p_lat_min, double const& p_lon_max, double const& p_lat_max);

    explicit ReachPolygon2(std::vector<std::shared_ptr<ReachPolygon2>> const& vec_polygons);

    std::vector<Vertex> vec_vertices;

    void print_info() const;

    /// Computes the bounding box of the polygon.
    void compute_bounding_box();

    void sort_vertices_bottom_left_first();

    /// Convexify the polygon.
    /// @note using Andrew monotone chain, see <a href="https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain">
    void convexify();

    void intersect_halfspace(double a1, double a2, double b);

    void minkowski_sum(std::shared_ptr<ReachPolygon2>& other);

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

    inline std::vector<Vertex> const& vertices() const {
        return vec_vertices;
    }

    inline Vertex const& get_vertex_with_cyclic_index(size_t i) const {
        assert(i >= 0);
        return vec_vertices[i % vec_vertices.size()];
    }

    inline void add_vertex(double x, double y) {
        auto vertex = Vertex{x, y};
        vec_vertices.emplace_back(vertex);
        update_bounding_box(vertex);
        _sorting_state = unsorted;
    }

    inline void add_vertex(Vertex vertex) {
        vec_vertices.emplace_back(vertex);
        update_bounding_box(vertex);
        _sorting_state = unsorted;
    }

    inline void add_polygon(std::shared_ptr<ReachPolygon2> const& polygon) {
        vec_vertices.insert(vec_vertices.end(), polygon->vec_vertices.begin(), polygon->vec_vertices.end());
        compute_bounding_box();
        _sorting_state = unsorted;
    }

    /// Updates the bounding box of the polygon.
    inline void update_bounding_box(Vertex const& vertex) {
        _box = {std::min(p_lon_min(), vertex.p_lon()),
                std::min(p_lat_min(), vertex.p_lat()),
                std::max(p_lon_max(), vertex.p_lon()),
                std::max(p_lat_max(), vertex.p_lat())};
    };

    inline void linear_mapping(double a11, double a12, double a21, double a22) {
        convexify();
        for (auto& vertex: vec_vertices) {
            double tmp = a11 * vertex.x + a12 * vertex.y;
            vertex.y = a21 * vertex.x + a22 * vertex.y;
            vertex.x = tmp;
        }
        _sorting_state = ccw;
        _remove_duplicated_vertices();
    }

    inline std::shared_ptr<ReachPolygon2> clone() const {
        return std::make_shared<ReachPolygon2>(vec_vertices);
    }

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