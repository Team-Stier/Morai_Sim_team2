#ifndef HYBRID_PATH_TRACKING_PATH_SOURCE_MANAGER_HPP_
#define HYBRID_PATH_TRACKING_PATH_SOURCE_MANAGER_HPP_

#include <cstdint>

#include "hybrid_path_tracking/control_types.hpp"

namespace hybrid_path_tracking {

enum class PathSource : uint8_t { GLOBAL = 0, VISION = 1, AVOIDANCE = 2 };

struct PathSourceConfig {
  double min_confidence{0.7};
  double min_forward_length_m{10.0};
  double stale_sec{0.2};
};

struct TimedPath {
  Path2D path;
  double stamp_sec;
  double forward_length_m;
};

struct PathSelectionContext {
  bool avoidance_active;
  bool gps_blackout;
  double now_sec;
};

struct SelectedPath {
  bool valid;
  PathSource source;
  Path2D path;
};

class PathSourceManager {
 public:
  explicit PathSourceManager(PathSourceConfig config);

  void updateGlobal(const TimedPath& value);
  void updateVision(const TimedPath& value);
  void updateAvoidance(const TimedPath& value);

  SelectedPath select(const PathSelectionContext& context) const;

 private:
  bool isValid(const TimedPath& value, double now_sec) const;
  SelectedPath selected(const TimedPath& value, PathSource source) const;
  SelectedPath invalid() const;

  PathSourceConfig config_;
  bool config_valid_{false};
  TimedPath global_{};
  TimedPath vision_{};
  TimedPath avoidance_{};
  bool has_global_{false};
  bool has_vision_{false};
  bool has_avoidance_{false};
};

}  // namespace hybrid_path_tracking

#endif  // HYBRID_PATH_TRACKING_PATH_SOURCE_MANAGER_HPP_
