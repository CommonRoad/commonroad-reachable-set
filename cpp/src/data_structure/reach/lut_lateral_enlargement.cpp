#include "reachset/data_structure/reach/lut_lateral_enlargement.hpp"

namespace reach {

LUTLateralEnlargement::LUTLateralEnlargement(std::vector<double> x_values) {
    for (const auto x : x_values) {
        auto it = this->lut_lat_enlargement_.end();
        this->lut_lat_enlargement_.insert(it, std::pair <double, double> (x, 0.0));
    }
}

std::map<double,
        double>::const_iterator LUTLateralEnlargement::longitudinalPositionHigherOrEqual(double x_hi) const {
    // get the first lateral position which higher or equal than x_hi
    auto it_hi = this->lut_lat_enlargement_.lower_bound(x_hi);
    if (it_hi == this->lut_lat_enlargement_.end()) {
        std::cout << "x_hi: " << x_hi << std::endl;
        // first and all other longitudinal positions are higher than y_hi
        throw std::invalid_argument("<LUTLateralEnlargement/maximumEnlargementInPositionRange> Longitudinal position is "
                                    "outside of look-up table.");
    }
    return it_hi;
}

std::map<double,
        double>::const_iterator LUTLateralEnlargement::longitudinalPositionLowerOrEqual(double x_lo) const {
    // get the first lateral position which is guaranteed to be higher than x_lo
    auto it_lo = this->lut_lat_enlargement_.upper_bound(x_lo);
    if (it_lo == this->lut_lat_enlargement_.begin()) {
        // first and all other longitudinal positions are higher than x_lo
        std::cout << "x_lo: " << x_lo << std::endl;
        throw std::invalid_argument("<LUTLateralEnlargement/maximumEnlargementInPositionRange> Longitudinal position "
                                    "is outside of look-up table.");
    } else {
        // found higher value, the previous position is equal or less than x_lo
        it_lo--;
    }
    return it_lo;
}

void LUTLateralEnlargement::insert(double x_lo, double x_hi, double dy) {
    auto it_lo = this->longitudinalPositionLowerOrEqual(x_lo);
    auto it_hi = this->longitudinalPositionHigherOrEqual(x_hi);
    it_hi++;
    for(auto it = it_lo; it != it_hi; ++it) {
        if(fabs((*it).second) < fabs(dy)) {
            this->lut_lat_enlargement_[it->first] = dy;
        }
    }
}


double LUTLateralEnlargement::maximumEnlargementInPositionRange(double x_lo, double x_hi) const {
    // get the first lateral position which is guaranteed to be higher than x_lo
    auto it_lo = this->longitudinalPositionLowerOrEqual(x_lo);
    // get the first lateral position which higher or equal than x_hi
    auto it_hi = this->longitudinalPositionHigherOrEqual(x_hi);
    it_hi++;
    std::vector<double> lateral_enlargements;
    for(auto it = it_lo; it != it_hi; ++it) {
        lateral_enlargements.push_back((*it).second);
    }
    return *(std::max_element(lateral_enlargements.begin(), lateral_enlargements.end()));
}
}