#include "reachset/semantic/data_structure/semantic_model.hpp"
#include "reachset/common/utility/shared_using.hpp"

using namespace reach;

SemanticModel::SemanticModel(pybind11::object const& obj_semantic_model_py) {
    for (auto const& obj: obj_semantic_model_py.attr("list_regions")) {
        vec_regions.emplace_back(make_shared<Region>(obj));
    }

    auto dict_time_step_to_position_intervals = obj_semantic_model_py.attr("dict_time_step_to_position_intervals");
    for (auto const& time_step: dict_time_step_to_position_intervals) {
        for (auto const& dir: dict_time_step_to_position_intervals[time_step]){
            for(auto const& position_interval_py: dict_time_step_to_position_intervals[time_step][dir]){
                map_time_step_to_position_intervals[time_step.cast<int>()][dir.cast<string>()].emplace_back(
                        make_shared<PositionInterval>(position_interval_py)
                        );
            }
        }
    }
}