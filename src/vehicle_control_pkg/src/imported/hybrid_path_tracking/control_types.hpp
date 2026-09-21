#ifndef HYBRID_PATH_TRACKING_CONTROL_TYPES_HPP_
#define HYBRID_PATH_TRACKING_CONTROL_TYPES_HPP_

#include <string>
#include <vector>

namespace hybrid_path_tracking {

struct Point2D {
  double x;
  double y;
};

struct VehicleState2D {
  double x;
  double y;
  double yaw;
  double speed_kph;
};

struct Path2D {
  std::vector<Point2D> points;
  double confidence;
  int source;
};

struct ControllerCandidate {
  bool valid;
  double steering_angle_rad;
  double cross_track_error_m;
  double heading_error_rad;
  double path_curvature;
  bool saturated;
  std::string controller;
};

struct PurePursuitConfig {
  double wheelbase_m{3.0};
  double max_steering_rad{40.0 * 3.14159265358979323846 / 180.0};
  double min_lookahead_m{3.0};
  double max_lookahead_m{15.0};
  double lookahead_time_sec{0.15};
  // Required maximum positive vehicle-frame x reach, not accumulated arc length.
  double min_forward_path_m{5.0};
};

}  // namespace hybrid_path_tracking

#endif  // HYBRID_PATH_TRACKING_CONTROL_TYPES_HPP_
