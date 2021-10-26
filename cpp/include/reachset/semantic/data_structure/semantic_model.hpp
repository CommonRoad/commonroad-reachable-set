#pragma once

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include "reachset/common/utility/shared_include.hpp"
#include "reachset/common/data_structure/configuration.hpp"
#include "reachset/semantic/data_structure/traffic_rule.hpp"
#include "reachset/semantic/data_structure/region.hpp"
#include "reachset/semantic/data_structure/position_interval.hpp"

namespace reach {
/// Class to represent the semantic model of a given CommonRoad scenario.
class SemanticModel {
public:
    ConfigurationPtr config;
    std::vector<RegionPtr> vec_regions;
    std::map<int, std::map<std::string, std::vector<PositionIntervalPtr>>> map_time_step_to_position_intervals;

    SemanticModel() = default;

    explicit SemanticModel(pybind11::object const& obj_semantic_model_py);
};

using SemanticModelPtr = shared_ptr<SemanticModel>;
}