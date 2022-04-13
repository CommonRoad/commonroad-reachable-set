#pragma once

#include "reachset/utility/shared_include.hpp"

namespace reach {
/// Line segment used in geometric operations.
/// A line segment is formed with two vertices.
struct ReachLine {
    double p_lon_min;
    double p_lat_min;
    double p_lon_max;
    double p_lat_max;

    ReachLine() = default;

    /// Constructor of ReachLine.
    /// @param p_lon_min minimum longitudinal position
    /// @param p_lat_min minimum lateral position
    /// @param p_lon_max maximum longitudinal position
    /// @param p_lat_max maximum lateral position
    ReachLine(double p_lon_min, double p_lat_min, double p_lon_max, double p_lat_max);

    bool operator==(ReachLine const& other) const;
};

using ReachLinePtr = std::shared_ptr<ReachLine>;
}