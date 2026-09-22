#ifndef HYBRID_PATH_TRACKING_PURE_PURSUIT_HPP_
#define HYBRID_PATH_TRACKING_PURE_PURSUIT_HPP_

#include "hybrid_path_tracking/control_types.hpp"

namespace hybrid_path_tracking {

class PurePursuit {
 public:
  explicit PurePursuit(PurePursuitConfig config);

  ControllerCandidate calculate(const VehicleState2D& state,
                                const Path2D& path) const;

 private:
  PurePursuitConfig config_;
};

}  // namespace hybrid_path_tracking

#endif  // HYBRID_PATH_TRACKING_PURE_PURSUIT_HPP_
