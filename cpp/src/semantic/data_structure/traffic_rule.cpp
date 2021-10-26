#include "reachset/semantic/data_structure/traffic_rule.hpp"
#include "reachset/common/utility/shared_using.hpp"

using namespace reach;

TrafficRule::TrafficRule(pybind11::object const& obj_traffic_rule_py) {
    vec_traffic_rules = obj_traffic_rule_py.attr("list_traffic_rules").cast<vector<string>>();
    vec_specifications_global = obj_traffic_rule_py.attr("list_specifications_global").cast<vector<string>>();
    vec_specifications_smv = obj_traffic_rule_py.attr("list_specifications_smv").cast<vector<string>>();
    vec_specifications_mtl = obj_traffic_rule_py.attr("list_specifications_mtl").cast<vector<string>>();
    vec_identifiers_smv = obj_traffic_rule_py.attr("list_specifications_mtl").cast<vector<string>>();
    set_ids_lanelets_in_specifications = obj_traffic_rule_py.attr("set_ids_lanelets_in_specifications").cast<set<int>>();
}
