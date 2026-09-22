#ifndef HYBRID_PATH_TRACKING_PATH_GEOMETRY_HPP_
#define HYBRID_PATH_TRACKING_PATH_GEOMETRY_HPP_

#include <vector>

#include "hybrid_path_tracking/control_types.hpp"

namespace hybrid_path_tracking {

struct CurvatureResult {
  bool valid;
  double max_abs_curvature;
};

CurvatureResult maximumAbsoluteCurvature(
    const std::vector<Point2D>& points);

}  // namespace hybrid_path_tracking

#endif  // HYBRID_PATH_TRACKING_PATH_GEOMETRY_HPP_
