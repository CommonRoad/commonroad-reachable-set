#include "reachset/utility/enlargement.hpp"

namespace reach {

/**
 * Inverts a given curvature value to obtain the corresponding radius
 * @param curvature
 * @return radius (= 1/curvature)
 */
static double curvatureRadius(const double curvature) {
    double curvature_radius;
    if(fabs(curvature) <= 1e-8) {
        curvature_radius = 1e8;
    }
    else {
        curvature_radius = 1 / fabs(curvature);
    }
    return curvature_radius;
}


/**
 * Returns the value for longitudinal enlargement of the rectangle for a given curvature range using the precomputed
 * Look-up table
 * @param min_curvature minimum of curvature range
 * @param max_curvature maximum of curvature range
 * @param lut_lon_enlargement look-up table longitudinal enlargement
 * @param y_lo
 * @param y_hi
 * @return value for enlargement in longitudinal direction
 */
static double computeLongitudinalEnlargement(
        const double min_curvature,
        const double max_curvature,
        const LUTLongitudinalEnlargement& lut_lon_enlargement,
        const double y_lo,
        const double y_hi) {
    double dx;
    if (min_curvature < 0. && max_curvature < 0.) {
        dx = lut_lon_enlargement.maximumEnlargementInCurvatureRange(
                -y_lo, -max_curvature, -min_curvature);
    } else if (min_curvature < 0. && max_curvature >= 0.) {
        double dx_lo = lut_lon_enlargement.maximumEnlargementInCurvatureRange(
                -y_lo, 0.0, -min_curvature);
        double dx_hi = lut_lon_enlargement.maximumEnlargementInCurvatureRange(
                y_hi, 0.0, max_curvature);
        dx = std::max(dx_lo, dx_hi);
    } else if (min_curvature >= 0. && max_curvature >= 0.) {
        dx = lut_lon_enlargement.maximumEnlargementInCurvatureRange(
                y_hi, min_curvature, max_curvature);
    } else {
        throw std::invalid_argument("<computeEnlargement> Minimum and maximum curvature of "
                                    "coordinate system are invalid.");
    }
    return dx;
}


/**
 * Returns the value for lateral enlargement of the rectangle for a given curvature range
 * @param min_curvature minimum of curvature range
 * @param max_curvature maximum of curvature range
 * @param l length = circle_distance (ref point REAR), length = circle_distance/2 (ref point CENTER)
 * @param y_lo
 * @param y_hi
 * @return tuple of values (y_lo, y_hi) for enlargement in lateral direction
 */
static std::tuple<double, double> computeLateralEnlargement(
        const double min_curvature,
        const double max_curvature,
        const double l,
        const double y_lo,
        const double y_hi) {

    auto dy = [](const double l, const double r) {
        return sqrt(pow(r, 2) + pow(l, 2)) - r;
    };

    double dy_lo, dy_hi;
    if (min_curvature < 0. && max_curvature < 0.) {
        dy_lo = 0.0;
        dy_hi = dy(l, curvatureRadius(min_curvature) + y_lo);
    } else if (min_curvature < 0. && max_curvature >= 0.) {
        dy_hi = dy(l, curvatureRadius(min_curvature) + y_lo);
        dy_lo = dy(l, curvatureRadius(max_curvature) - y_hi);
    } else if (min_curvature >= 0. && max_curvature >= 0.) {
        dy_lo = dy(l, curvatureRadius(max_curvature) - y_hi);
        dy_hi = 0.0;
    } else {
        throw std::invalid_argument("<computeEnlargement> Minimum and maximum curvature of "
                                    "coordinate system are invalid.");
    }
    return std::make_tuple(dy_lo, dy_hi);
}


/**
 *
 * @param cosy Curvilinear Coordinate System
 * @param lut_lon_enlargement Look-up table with precomputed longitudinal enlargements for different curvature intervals
 * @param reference_point Reference point of the vehicle (CENTER or REAR)
 * @param circle_distance distance between first and third circle
 * @param x_lo minimum longitudinal coordinate of rectangle
 * @param x_hi maximum longitudinal coordinate of rectangle
 * @param y_lo minimum lateral coordinate of rectangle
 * @param y_hi maximum lateral coordinate of rectangle
 * @return tuple of values (dx, dy_lo, dy_hi) for longitudinal dx and lateral dy_lo, dy_hi enlargement
 */
static std::tuple<double, double, double> computeEnlargement(
        const geometry::CurvilinearCoordinateSystem &cosy,
        const LUTLongitudinalEnlargement& lut_lon_enlargement,
        const ReferencePoint reference_point,
        const double circle_distance,
        const double x_lo,
        const double x_hi,
        const double y_lo,
        const double y_hi) {

    double dx = computeLongitudinalEnlargement(
            cosy.minimumCurvature(), cosy.maximumCurvature(), lut_lon_enlargement, y_lo, y_hi);

    double min_curvature, max_curvature, l;
    switch (reference_point) {
        case ReferencePoint::CENTER:
            l = circle_distance / 2.0;
            std::tie(min_curvature, max_curvature) = cosy.curvatureRange(x_lo - dx, x_hi + dx);
            break;
        case ReferencePoint::REAR:
            l = circle_distance;
            std::tie(min_curvature, max_curvature) = cosy.curvatureRange(x_lo, x_hi + dx);
            break;
        default:
            throw std::invalid_argument("<computeEnlargement> Reference point for reachable set computation"
                                        "is invalid.");
    }

    double dy_lo, dy_hi;
    dx = computeLongitudinalEnlargement(
            min_curvature, max_curvature, lut_lon_enlargement, y_lo, y_hi);
    std::tie(dy_lo, dy_hi) = computeLateralEnlargement(min_curvature, max_curvature, l, y_lo, y_hi);
    return std::make_tuple(dx, dy_lo, dy_hi);
}


std::tuple<double, double, double, double> computeEnlargedRectangle(
        const geometry::CurvilinearCoordinateSystem &cosy,
        const LUTLongitudinalEnlargement& lut_lon_enlargement,
        const ReferencePoint reference_point,
        const double circle_distance,
        std::tuple<double, double, double, double> coords) {
    double dx, dy_lo, dy_hi;
    std::tie(dx, dy_lo, dy_hi) = computeEnlargement(
            cosy, lut_lon_enlargement, reference_point, circle_distance,
            std::get<0>(coords), std::get<1>(coords),
            std::get<2>(coords), std::get<3>(coords));

    std::tuple<double, double, double, double> enlarged_coords;
    switch (reference_point) {
        case ReferencePoint::CENTER:
            enlarged_coords = std::make_tuple(
                    std::get<0>(coords) - dx,
                    std::get<1>(coords) + dx,
                    std::get<2>(coords) - dy_lo,
                    std::get<3>(coords) + dy_hi);
            break;
        case ReferencePoint::REAR:
            enlarged_coords = std::make_tuple(
                    std::get<0>(coords),
                    std::get<1>(coords) + dx,
                    std::get<2>(coords) - dy_lo,
                    std::get<3>(coords) + dy_hi);
            break;
        default:
            throw std::invalid_argument("<computeEnlargedRectangle> Reference point for reachable set computation"
                                        "is invalid.");
    }
    return enlarged_coords;
}

}