#pragma once

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include "reachset/common/utility/shared_include.hpp"
#include "reachset/common/data_structure/configuration.hpp"

namespace reach {
/// Class to hold adopted traffic rules.
class TrafficRule {
public:
    ConfigurationPtr config;
    std::vector<std::string> vec_traffic_rules{};
    std::vector<std::string> vec_specifications_global{};
    std::vector<std::string> vec_specifications_smv{};
    std::vector<std::string> vec_specifications_mtl{};

    std::vector<std::string> vec_identifiers_smv{};
    std::set<int> set_ids_lanelets_in_specifications{};

    TrafficRule() = default;

    //explicit TrafficRule(ConfigurationPtr config);

    explicit TrafficRule(pybind11::object const& obj_traffic_rule_py);
};

using TrafficRulePtr = std::shared_ptr<TrafficRule>;
}