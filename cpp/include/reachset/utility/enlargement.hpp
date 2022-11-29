#pragma once

#include "geometry/curvilinear_coordinate_system.h"
#include "reachset/utility/shared_include.hpp"
#include "reachset/utility/lut_longitudinal_enlargement.hpp"
#include "reachset/data_structure/configuration.hpp"

namespace reach {

/// Computes enlargement of a given rectangle and returns the coordinates of the enlarged rectangle.
/// The enlargement is computed to consider the occupancy of the full vehicle (approximated by three circles) in the
/// reachable set computation.
/// @note Computation taken from "Manzinger et al. (2020): Using reachable sets for trajectory planning of automated
/// vehicles, IEEE Transactions on Intelligent Vehicles" (see Appendix B for full approach)
/// @param cosy Curvilinear Coordinate System
/// @param lut_lon_enlargement Look-up table with precomputed longitudinal enlargements for different curvature intervals
/// @param reference_point Reference point of the vehicle (CENTER or REAR)
/// @param circle_distance distance between first and third circle
/// @param coords coordinates (x_min, x_max, y_min, y_max) of the original rectangle
/// @return coordinates (x_min, x_max, y_min, y_max) of the enlarged rectangle
std::tuple<double, double, double, double> computeEnlargedRectangle(const geometry::CurvilinearCoordinateSystem& cosy,
                                                                    const LUTLongitudinalEnlargement& lut_lon_enlargement,
                                                                    ReferencePoint reference_point,
                                                                    double circle_distance,
                                                                    std::tuple<double, double, double, double> coords);

}
