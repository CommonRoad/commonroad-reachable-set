#pragma once

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include "reachset/common/utility/shared_include.hpp"
#include "reachset/common/data_structure/reach_polygon.hpp"

namespace reach {
/// Class to represent position intervals in which a set of propositions hold.
class Region {
public:
    std::vector<int> vec_ids_lanelets{};
    std::set<std::string> set_propositions{};
    ReachPolygonPtr polygon_cart;
    ReachPolygonPtr polygon_cvln;
    std::vector<ReachPolygonPtr> vec_aabbs;

    Region() = default;

    explicit Region(pybind11::handle region_py);

    bool excludes(ReachPolygonPtr const& rectangle, std::string const& coordinate = "CVLN") const;
};

using RegionPtr = std::shared_ptr<Region>;
}