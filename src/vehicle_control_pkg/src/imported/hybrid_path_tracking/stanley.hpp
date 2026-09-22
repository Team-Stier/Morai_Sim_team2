#ifndef HYBRID_PATH_TRACKING_STANLEY_HPP_
#define HYBRID_PATH_TRACKING_STANLEY_HPP_

#include "hybrid_path_tracking/control_types.hpp"

namespace hybrid_path_tracking {

struct StanleyConfig {
  double wheelbase_m{3.0};
  double heading_gain{1.0};
  double cross_track_gain{1.2};
  double softening_mps{0.8};
  double max_steering_rad{40.0 * 3.14159265358979323846 / 180.0};
  // Required maximum positive vehicle-frame x reach, not accumulated arc length.
  double min_path_length_m{5.0};
};

StanleyConfig defaultStanleyConfig();

class Stanley {
 public:
  explicit Stanley(StanleyConfig config);

  ControllerCandidate calculate(const VehicleState2D& state,
                                const Path2D& path) const;

 private:
  StanleyConfig config_;
};

}  // namespace hybrid_path_tracking

#endif  // HYBRID_PATH_TRACKING_STANLEY_HPP_
