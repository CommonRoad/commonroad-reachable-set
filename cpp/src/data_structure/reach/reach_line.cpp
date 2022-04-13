#include "reachset/data_structure/reach/reach_line.hpp"

using namespace reach;

ReachLine::ReachLine(double p_lon_min, double p_lat_min, double p_lon_max, double p_lat_max) :
        p_lon_min(p_lon_min), p_lat_min(p_lat_min), p_lon_max(p_lon_max), p_lat_max(p_lat_max) {}

bool ReachLine::operator==(ReachLine const& other) const {
    if (this->p_lon_min == other.p_lon_min and this->p_lon_max == other.p_lon_max and
        this->p_lat_min == other.p_lat_min and this->p_lat_max == other.p_lat_max)
        return true;

    return false;
}