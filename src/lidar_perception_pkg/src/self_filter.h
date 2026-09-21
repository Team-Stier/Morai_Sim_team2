#pragma once
#include "detector.h"

namespace lidar_perception {
// Local preprocessing bounds in the original lidar_link frame, before leveling.
struct SelfFilterConfig {
  bool enabled=true;
  double x_min=-2.790, x_max=1.845, y_min=-.946, y_max=.946;
  double z_min=-1.5, z_max=.934;
  void validate() const;
};
// Mask rather than erase: every index must still identify the original raw record.
Cloud maskSelfReturns(const Cloud& input, const SelfFilterConfig& config);
}
