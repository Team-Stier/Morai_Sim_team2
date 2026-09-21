#include "self_filter.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lidar_perception {
void SelfFilterConfig::validate() const {
  for (double v : {x_min,x_max,y_min,y_max,z_min,z_max})
    if (!std::isfinite(v)) throw std::invalid_argument("nonfinite self filter bound");
  if (x_min>=x_max || y_min>=y_max || z_min>=z_max)
    throw std::invalid_argument("invalid self filter bounds");
}
Cloud maskSelfReturns(const Cloud& input, const SelfFilterConfig& config) {
  config.validate();
  Cloud result=input;
  if (!config.enabled) return result;
  for (auto& p : result) {
    // Inclusive bounds expressed as floats like the incoming PointCloud2 XYZ.
    if (p.x>=static_cast<float>(config.x_min) && p.x<=static_cast<float>(config.x_max) &&
        p.y>=static_cast<float>(config.y_min) && p.y<=static_cast<float>(config.y_max) &&
        p.z>=static_cast<float>(config.z_min) && p.z<=static_cast<float>(config.z_max)) {
      p.x=p.y=p.z=std::numeric_limits<float>::quiet_NaN();
      result.is_dense=false;
    }
  }
  return result;
}
}
