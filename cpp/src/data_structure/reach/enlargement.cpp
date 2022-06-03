#include "reachset/data_structure/reach/enlargement.hpp"

namespace reach {

    static double curvatureRadius(const double curvature) {
        double curvature_radius;
        if(fabs(curvature) <= 1e-8) {
            curvature_radius = 1e8;
        }
        else {
            curvature_radius = 1 / fabs(curvature);
        }
        return curvature_radius;
    };


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


    static std::tuple<double, double, double> computeEnlargement(
            const geometry::CurvilinearCoordinateSystem &cosy,
            const LUTLongitudinalEnlargement& lut_lon_enlargement,
            const ReferencePoint reference_point,
            const double wheelbase,
            const double x_lo,
            const double x_hi,
            const double y_lo,
            const double y_hi) {

        double dx = computeLongitudinalEnlargement(
                cosy.minimumCurvature(), cosy.maximumCurvature(), lut_lon_enlargement, y_lo, y_hi);

        double min_curvature, max_curvature, l;
        switch (reference_point) {
            case ReferencePoint::CENTER:
                l = wheelbase / 2.0;
                std::tie(min_curvature, max_curvature) = cosy.curvatureRange(x_lo - dx, x_hi + dx);
                break;
            case ReferencePoint::REAR:
                l = wheelbase;
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


    static std::tuple<double, double, double, double> computeEnlargedRectangle(
            const ReferencePoint reference_point,
            double dx, double dy_lo, double dy_hi,
            std::tuple<double, double, double, double> coords) {

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


    std::tuple<double, double, double, double> computeEnlargedRectangle(
            const geometry::CurvilinearCoordinateSystem &cosy,
            const LUTLongitudinalEnlargement& lut_lon_enlargement,
            const ReferencePoint reference_point,
            const double wheelbase,
            std::tuple<double, double, double, double> coords) {
        double dx, dy_lo, dy_hi;
        std::tie(dx, dy_lo, dy_hi) = computeEnlargement(
                cosy, lut_lon_enlargement, reference_point, wheelbase,
                std::get<0>(coords), std::get<1>(coords),
                std::get<2>(coords), std::get<3>(coords));
        return computeEnlargedRectangle(reference_point, dx, dy_lo, dy_hi, coords);
    }

}