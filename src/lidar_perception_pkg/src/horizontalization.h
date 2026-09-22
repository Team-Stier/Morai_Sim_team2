#pragma once

#include "detector.h"
#include <Eigen/Geometry>
#include <cstdint>
#include <deque>

namespace lidar_perception {
// Port of horizontal_pkg/Paik level_rotation; see docs/horizontalization.md.
// The temporary level coordinates have their origin at the LiDAR, not base_link.
Eigen::Matrix3d levelRotation(const Eigen::Quaterniond& map_from_body,
                              const Eigen::Quaterniond& body_from_lidar);
Cloud rotateCloud(const Cloud& input, const Eigen::Matrix3d& rotation);
Box boxInSensorFrame(const Box& level_box, const Eigen::Matrix3d& level_from_lidar);

class AttitudeHistory {
  struct Sample { uint64_t stamp; Eigen::Quaterniond orientation; };
  std::deque<Sample> samples_;
public:
  void clear() { samples_.clear(); }
  bool insert(uint64_t stamp, const Eigen::Quaterniond& orientation, double history_sec);
  bool interpolate(uint64_t stamp, double max_gap_sec, Eigen::Quaterniond& result) const;
};
}  // namespace lidar_perception
