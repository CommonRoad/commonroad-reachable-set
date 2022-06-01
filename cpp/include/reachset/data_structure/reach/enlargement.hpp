#pragma once

#include "geometry/curvilinear_coordinate_system.h"
#include "reachset/utility/shared_include.hpp"
#include "reachset/data_structure/reach/lut_longitudinal_enlargement.hpp"
#include "reachset/data_structure/configuration.hpp"

namespace reach {

// TODO add docstring/comments

static double curvatureRadius(const double curvature);

static double computeLongitudinalEnlargement(const double min_curvature, const double max_curvature,
                                             const LUTLongitudinalEnlargement& lut_lon_enlargement, const double y_lo,
                                             const double y_hi);

static std::tuple<double, double> computeLateralEnlargement(const double min_curvature, const double max_curvature,
                                                            const double l, const double y_lo, const double y_hi);

static std::tuple<double, double, double> computeEnlargement(const geometry::CurvilinearCoordinateSystem &cosy,
                                                             const LUTLongitudinalEnlargement& lut_lon_enlargement,
                                                             const ReferencePoint reference_point,
                                                             const double wheelbase, const double x_lo,
                                                             const double x_hi, const double y_lo, const double y_hi);

// TODO Check if this function can be removed?
static std::tuple<double, double> computeEnlargedLongitudinalCoordObstacle(const ReferencePoint reference_point,
                                                                           double dx, double x_lo, double x_hi);

// TODO Check if this function can be removed?
static std::tuple<double, double, double, double> computeEnlargedObstacle(const ReferencePoint reference_point,
                                                                          double dx, double dy_lo, double dy_hi,
                                                                          std::tuple<double, double, double, double> coords);

static std::tuple<double, double> computeEnlargedLongitudinalCoordRectangle(const ReferencePoint reference_point,
                                                                            double dx, double x_lo, double x_hi);

static std::tuple<double, double, double, double> computeEnlargedRectangle(const ReferencePoint reference_point,
                                                                           double dx, double dy_lo, double dy_hi,
                                                                           std::tuple<double, double, double, double> coords);

static std::tuple<double, double, double, double> computeEnlargedRectangle(const geometry::CurvilinearCoordinateSystem &cosy,
                                                                           const LUTLongitudinalEnlargement& lut_lon_enlargement,
                                                                           const ReferencePoint reference_point,
                                                                           const double wheelbase,
                                                                           std::tuple<double, double, double, double> coords);

} // namespace reach
