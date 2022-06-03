#pragma once

#include "geometry/curvilinear_coordinate_system.h"
#include "reachset/utility/shared_include.hpp"
#include "reachset/data_structure/reach/lut_longitudinal_enlargement.hpp"
#include "reachset/data_structure/configuration.hpp"

namespace reach {

// TODO add docstring/comments

static double curvatureRadius(double curvature);

static double computeLongitudinalEnlargement(double min_curvature, double max_curvature,
                                             const LUTLongitudinalEnlargement& lut_lon_enlargement, double y_lo,
                                             double y_hi);

static std::tuple<double, double> computeLateralEnlargement(double min_curvature, double max_curvature,
                                                            double l, double y_lo, double y_hi);

static std::tuple<double, double, double> computeEnlargement(const geometry::CurvilinearCoordinateSystem &cosy,
                                                             const LUTLongitudinalEnlargement& lut_lon_enlargement,
                                                             ReferencePoint reference_point,
                                                             double wheelbase, double x_lo,
                                                             double x_hi, double y_lo, double y_hi);

static std::tuple<double, double, double, double> computeEnlargedRectangle(ReferencePoint reference_point,
                                                                           double dx, double dy_lo, double dy_hi,
                                                                           std::tuple<double, double, double, double> coords);

std::tuple<double, double, double, double> computeEnlargedRectangle(const geometry::CurvilinearCoordinateSystem &cosy,
                                                                           const LUTLongitudinalEnlargement& lut_lon_enlargement,
                                                                           ReferencePoint reference_point,
                                                                           double wheelbase,
                                                                           std::tuple<double, double, double, double> coords);

} // namespace reach
