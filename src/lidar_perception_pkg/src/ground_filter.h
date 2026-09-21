#pragma once
#include "detector.h"
namespace lidar_perception {
// Local detection preprocessing only. No persistent map or free-space claim.
struct GroundConfig {
  bool enabled=false;
  double cell_size=4, sample_size=.5, max_range=55;
  double expected_z=-1.9, seed_radius=8, seed_tolerance=.25;
  double max_slope_deg=18, fit_distance=.05, remove_distance=.07;
  double min_span=3, min_inlier_ratio=.6, continuity_distance=.08;
  int min_support=12, iterations=64;
  void validate() const;
};
struct GroundResult {
  Cloud cloud;
  size_t removed=0, supported_cells=0;
};
GroundResult removeGround(const Cloud& leveled, const GroundConfig& config);
}
