#pragma once

#include <omp.h>
#include <Eigen/Dense>
#include "reachset/utility/shared_include.hpp"

#include "geometry/curvilinear_coordinate_system.h"
#include "collision/collision_checker.h"
#include "collision/narrowphase/rectangle_aabb.h"
#include "collision/narrowphase/rectangle_obb.h"

#include "collision/time_variant_collision_object.h"
#include "reachset/common/data_structure/geometry_definition.hpp"

namespace reach {
using Polyline = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

/// Struct storing buffer strategies used in Boost.Geometry library.
struct BufferConfig {
    double buffer_distance;
    int const points_per_circle{5};
    bg::strategy::buffer::distance_symmetric<double> distance_strategy;
    bg::strategy::buffer::join_round join_strategy;
    bg::strategy::buffer::end_round end_strategy;
    bg::strategy::buffer::point_circle circle_strategy;
    bg::strategy::buffer::side_straight side_strategy;

    explicit BufferConfig(double const& buffer_distance);
};

BufferConfig create_buffer_configuration(double& radius_disc_vehicle);

/// Creates a collision checker with obstacles converted into curvilinear coordinate system.
/// @param vec_polylines_static vector of polylines describing static obstacles
/// @param map_time_step_to_vec_polylines_dynamic map from time step to vector of polylines describing dynamic obstacles
/// @param CLCS a curvilinear coordinate system object
/// @param radius_disc_vehicle radius of the three discs approximating the occupancy of the vehicle
/// @param num_omp_threads number of threads for parallel computing
/// @return pointer to collision checker
collision::CollisionCheckerPtr create_curvilinear_collision_checker(
        std::vector<Polyline> const& vec_polylines_static,
        std::map<int, std::vector<Polyline>> const& map_time_step_to_vec_polylines_dynamic,
        std::shared_ptr<geometry::CurvilinearCoordinateSystem> const& CLCS,
        double const& radius_disc_vehicle, int const& num_omp_threads);

/// Creates axis-aligned bounding boxes in curvilinear coordinate system from polylines in Cartesian coordinate system.
/// @param vec_polylines vector of polylines to be considered
/// @param CLCS a curvilinear coordinate system object
/// @param num_threads number of threads for parallel computing
/// @param buffer_config buffer config used in Boost.Geometry library
/// @return vector of pointers to rectangles
std::vector<collision::RectangleAABBPtr> create_curvilinear_aabbs_from_cartesian_polylines(
        std::vector<Polyline> const& vec_polylines,
        std::shared_ptr<geometry::CurvilinearCoordinateSystem> const& CLCS,
        int const& num_threads,
        BufferConfig const& buffer_config);

/// Converts an Eigen polyline into a Boost.Geometry polygon.
GeometryPolygon convert_polyline_to_geometry_polygon(Polyline const& polyline);

/// Dilates the input polygon with the given buffer config.
GeometryPolygon inflate_polygon(GeometryPolygon const& polygon, BufferConfig const& buffer_config);

/// Converts a Boost.Geometry polygon into an Eigen polyline.
Polyline convert_geometry_polygon_to_polyline(GeometryPolygon const& polygon);

/// Creates an axis-aligned bounding box from the input polyline.
collision::RectangleAABBPtr create_aabb_from_polyline(Polyline const& polyline);

/// Returns a tuple of the minimum/maximum coordinates of the input polyline in the longitudinal and lateral directions.
std::tuple<double, double, double, double>
obtain_extremum_coordinates_of_polyline(Polyline const& polyline);

/// Creates an axis-aligned bounding box from the input coordinates.
collision::RectangleAABBPtr create_aabb_from_coordinates(double const& p_lon_min, double const& p_lat_min,
                                                         double const& p_lon_max, double const& p_lat_max);

void print_vertices_polygon(std::vector<Polyline> const& vec_polylines_static);
}