#pragma once

#include "reachset/utility/shared_include.hpp"
#include <utility>

namespace reach {

struct cmpByFirst {
  bool operator()(const std::pair<double, double>& a, const std::pair<double, double>& b) const {
    return a.first < b.first;
  }
};

typedef std::map<std::pair < double, double>, double, cmpByFirst> LookUpTable;

// TODO add docstring/comments

class LUTLongitudinalEnlargement {
 public:
  explicit LUTLongitudinalEnlargement(std::map<double, std::map<std::pair<double, double>, double>> lut);
  std::map<double, LookUpTable>::const_iterator lateralPositionHigherOrEqual(double y_hi) const;
  double maximumEnlargementInCurvatureRange(double y, double min_curvature, double max_curvature) const;

 private:
  std::map<double, LookUpTable> lut_lon_enlargement_;

};

typedef std::shared_ptr<const LUTLongitudinalEnlargement> LUTLongitudinalEnlargementConstPtr;

} // namespace reach
