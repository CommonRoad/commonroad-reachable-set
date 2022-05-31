#include "reachset/data_structure/reach/lut_longitudinal_enlargement.hpp"

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

}
