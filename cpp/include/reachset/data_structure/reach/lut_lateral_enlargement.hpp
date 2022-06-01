#pragma once

#include "reachset/utility/shared_include.hpp"
#include <utility>

namespace reach {

// TODO add docstring/comments

class LUTLateralEnlargement {
public:
    explicit LUTLateralEnlargement(std::vector<double> x_values);
    std::map<double, double>::const_iterator longitudinalPositionHigherOrEqual(double x_hi) const;
    std::map<double, double>::const_iterator longitudinalPositionLowerOrEqual(double x_lo) const;
    void insert(double x_lo, double x_hi, double dy);
    double maximumEnlargementInPositionRange(double x_lo, double x_hi) const;

private:
    std::map<double, double> lut_lat_enlargement_;

};

} // namespace reach
