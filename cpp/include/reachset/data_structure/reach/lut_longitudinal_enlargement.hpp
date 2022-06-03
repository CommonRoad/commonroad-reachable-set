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


class LUTLongitudinalEnlargement {
 public:
  /**
   * Creates a Look-up Table storing precomputed values for the longitudinal enlargement of rectangles for different
   * curvature intervals
   * @param lut look-up table as map
   */
  explicit LUTLongitudinalEnlargement(std::map<double, std::map<std::pair<double, double>, double>> lut);

  /**
   * Returns the first lateral position which higher or equal than a given lateral position y_hi
   * @param y_hi
   * @return
   */
  std::map<double, LookUpTable>::const_iterator lateralPositionHigherOrEqual(double y_hi) const;

  /**
   * Returns the maximum longitudinal enlargement within a given curvature range and for a given lateral position y
   * @param y lateral position
   * @param min_curvature minimum of curvature range
   * @param max_curvature maximum of curvature range
   * @return longitudinal enlargement
   */
  double maximumEnlargementInCurvatureRange(double y, double min_curvature, double max_curvature) const;

 private:
  std::map<double, LookUpTable> lut_lon_enlargement_;

};

typedef std::shared_ptr<const LUTLongitudinalEnlargement> LUTLongitudinalEnlargementConstPtr;

} // namespace reach
