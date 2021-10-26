#include "reachset/semantic/data_structure/position_interval.hpp"
#include "reachset/common/utility/shared_using.hpp"

using namespace reach;

PositionInterval::PositionInterval(double const& p_min, double const& p_max, set<std::string> const& set_propositions) :
        p_min(p_min), p_max(p_max), set_propositions(set_propositions) {}

PositionInterval::PositionInterval(pybind11::handle obj_position_interval_py) {
    p_min = obj_position_interval_py.attr("p_min").cast<double>();
    p_max = obj_position_interval_py.attr("p_max").cast<double>();
    set_propositions = obj_position_interval_py.attr("set_propositions").cast<set<string>>();
}

bool PositionInterval::intersects(double const& p_min, double const& p_max) const {
    return not(p_min > this->p_max or p_max < this->p_min);
}
