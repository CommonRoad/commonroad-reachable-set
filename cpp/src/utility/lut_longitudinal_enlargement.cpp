#include "reachset/utility/lut_longitudinal_enlargement.hpp"

namespace reach {

LUTLongitudinalEnlargement::LUTLongitudinalEnlargement(
    std::map<double, std::map<std::pair<double, double>, double>> lut) {
  for (auto it = lut.begin(); it != lut.end(); it++) {
    LookUpTable lut_tmp;
    for(auto it_inner = it->second.begin(); it_inner != it->second.end(); it_inner++) {
      lut_tmp[it_inner->first] = it_inner->second;
    }
    this->lut_lon_enlargement_[it->first] = lut_tmp;
  }
}


std::map<double, LookUpTable>::const_iterator LUTLongitudinalEnlargement::lateralPositionHigherOrEqual(double y_hi) const {
    // get the first lateral position which higher or equal than y_hi
    auto it = this->lut_lon_enlargement_.lower_bound(y_hi);
    if (it == this->lut_lon_enlargement_.end()) {
        std::cout << "y_hi: " << y_hi << std::endl;
        // first and all other lateral positions are higher than y_hi
        throw std::invalid_argument("<lateralPositionHigherOrEqual> Lateral position is outside of look-up table.");
    }
    return it;
}


double LUTLongitudinalEnlargement::maximumEnlargementInCurvatureRange(double y,
                                                                      double min_curvature,
                                                                      double max_curvature) const {
    std::map<double, LookUpTable>::const_iterator it_lut;
    it_lut = this->lateralPositionHigherOrEqual(y);

    if (it_lut == this->lut_lon_enlargement_.end()) {
        std::cout << "y: " << y << " ,min_curvature: " << min_curvature << ", max_curvature: " << max_curvature << std::endl;
        throw std::invalid_argument("<maximumEnlargementInCurvatureRange> Lateral position is outside of look-up table.");
    }

    LookUpTable lut_curvatures = (*it_lut).second;
    // find interval of curvatures which contains min_curvature
    auto it_min_bound = lut_curvatures.upper_bound(std::make_pair(min_curvature, min_curvature));
    if (it_min_bound == lut_curvatures.begin()) {
        if (lut_curvatures.begin()->first.first > min_curvature) {
            // first and all other curvatures are higher than min_curvature
            throw std::invalid_argument("<maximumEnlargementInCurvatureRange> "
                                        "Minimum curvature is outside of look-up table.");
        }
    } else {
        // found higher value, the previous curvature interval contains min_curvature
        it_min_bound--;
    }

    // find interval of curvatures which contains max_curvature
    auto it_max_bound = lut_curvatures.lower_bound(std::make_pair(max_curvature, max_curvature));
    if (it_max_bound == lut_curvatures.begin()) {
        // first and all other curvatures are higher than max_curvature
        if (lut_curvatures.begin()->first.first < max_curvature) {
            std::cout << "max_curvature: " << max_curvature << std::endl;
            throw std::invalid_argument("<maximumEnlargementInCurvatureRange> "
                                        "Maximum curvature is outside of look-up table.");
        }
        else {
            it_max_bound++;
        }
    } else if (it_max_bound == lut_curvatures.end()) {
        // check if max_curvature is higher than maximum curvature in look-up table
        if (lut_curvatures.rbegin()->first.second < max_curvature) {
            std::cout << "max_curvature: " << max_curvature << std::endl;
            throw std::invalid_argument("<maximumEnlargementInCurvatureRange> "
                                        "Maximum curvature is outside of look-up table.");
        }
    }

    double max_longitudinal_enlargement = 0.0;
    if (it_min_bound == it_max_bound) {
        throw std::invalid_argument("<maximumEnlargementInCurvatureRange> "
                                    "Failure in curvature range computation.");
    }
    for(auto it = it_min_bound; it != it_max_bound; ++it) {
        if ((*it).second > max_longitudinal_enlargement) {
            max_longitudinal_enlargement = (*it).second;
        }
    }
    return max_longitudinal_enlargement;
}
}
