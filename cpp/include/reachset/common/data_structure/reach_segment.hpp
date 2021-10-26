#pragma once

#include "reachset/common/utility/shared_include.hpp"

namespace reach {
/// Line segment used in geometric operations.
/// A line segment segment is formed with two vertices.
struct ReachSegment {
    double p_lon_min;
    double p_lat_min;
    double p_lon_max;
    double p_lat_max;

    ReachSegment() = default;

    /// Constructor of ReachSegment.
    /// @param p_lon_min minimum longitudinal position
    /// @param p_lat_min minimum lateral position
    /// @param p_lon_max maximum longitudinal position
    /// @param p_lat_max maximum lateral position
    ReachSegment(double p_lon_min, double p_lat_min, double p_lon_max, double p_lat_max);

    bool operator==(ReachSegment const& other) const;
};

using ReachSegmentPtr = std::shared_ptr<ReachSegment>;
}