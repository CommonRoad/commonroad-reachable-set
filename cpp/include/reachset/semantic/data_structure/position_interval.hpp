#pragma once

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include "reachset/common/utility/shared_include.hpp"

namespace reach {
/// Class to represent position intervals in which a set of propositions hold.
class PositionInterval {
public:
    double p_min{};
    double p_max{};
    std::set<std::string> set_propositions{};

    PositionInterval() = default;

    PositionInterval(double const& p_min, double const& p_max, std::set<std::string> const& set_propositions);

    explicit PositionInterval(pybind11::handle obj_position_interval_py);

    bool intersects(double const& p_min, double const& p_max) const;
};

using PositionIntervalPtr = std::shared_ptr<PositionInterval>;
}
