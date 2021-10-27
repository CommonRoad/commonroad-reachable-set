#pragma once

#include "reachset/utility/geometry_definition.hpp"
#include "reachset/utility/shared_include.hpp"

namespace reach {
/// Polygon class used in reachset nodes and position rectangles.
/// When used to represent a reachset node, it is defined in the position-velocity domain, and can be used to represent
/// a polygon in either the longitudinal or the lateral direction; When used to represent a position, it is defined in
/// the longitudinal/lateral position domain.
/// @note Uses Boost.Geometry library.
class ReachPolygon {
private:
    GeometryPolygonPtr _polygon;
    std::tuple<double, double, double, double> _box;

public:
    ReachPolygon() = default;

    /// Constructor of ReachPolygon.
    /// @param vec_vertices vector of tuples representing the vertices
    explicit ReachPolygon(std::vector<std::tuple<double, double>> const& vec_vertices);

    explicit ReachPolygon(GeometryPolygon const& polygon, bool const& correct = true);

    std::shared_ptr<ReachPolygon> clone() const;

    inline GeometryPolygonPtr geometry_polygon() { return this->_polygon; }

    inline GeometryRing vertices() const { return this->_polygon->outer(); }

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

    inline bool empty() const { return vertices().empty(); }

    inline bool valid() const { return _polygon != nullptr; };

    /// Updates the bounding box of the polygon.
    void update_bounding_box();

    /// Intersects with the halfspace specified in the form of ax + by <= c.
    void intersect_halfspace(double const& a, double const& b, double const& c);

    /// Convexify the polygon.
    void convexify();

    /// Linear mapping in two dimensional plane.
    /// Computes the zero-input response of the system.
    /// @note see <a href="https://ieeexplore.ieee.org/abstract/document/8047450?casa_token=kfXwS_YEKoEAAAAA:k6TBp5W4m1BDxAZuMWI8S8T1I-fe38s3txJkzuNjiUMPv2T9Ccp1rzCmzNx8Z27VnnIgA4LU-Q">
    /// *Computing the Drivable Area of Autonomous Road Vehicles in Dynamic Road Scenes*</a> , Sec.IV.A.
    void linear_mapping(double const& a11, double const& a12, double const& a21, double const& a22);

    /// Computes the minkowski sum with the input polygon.
    /// @note Taken from Sebastian Söntges/Stefanie Manzinger's implementation.
    /// 5x faster than naively computing all new vertices and taking convex hull with Boost.Geometry.
    void minkowski_sum(std::shared_ptr<ReachPolygon> const& polygon_other);

    void print_vertices() const;

    bool intersects(std::shared_ptr<ReachPolygon> const& polygon_other) const;

    std::vector<std::shared_ptr<ReachPolygon>> intersection(std::shared_ptr<ReachPolygon> const& polygon_other) const;

    /// Creates a polygon from the given coordinates of a bounding box.
    /// @param p_lon_min minimum longitudinal position
    /// @param p_lat_min minimum lateral position
    /// @param p_lon_max maximum longitudinal position
    /// @param p_lat_max maximum lateral position
    static std::shared_ptr<ReachPolygon> from_rectangle_coordinates(double const& p_lon_min, double const& p_lat_min,
                                                                    double const& p_lon_max, double const& p_lat_max);

    /// Constructs a halfspace with the given parameters in the form of ax + by <= c.
    /// A rectangle is constructed to represent the halfspace.
    static std::shared_ptr<ReachPolygon>
    construct_halfspace_polygon(double const& a, double const& b, double const& c,
                                std::tuple<double, double, double, double> const& bounding_box);
};

using ReachPolygonPtr = std::shared_ptr<ReachPolygon>;

}