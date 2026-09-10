#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace lidar_perception {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
struct Config {
  double x_min=0, x_max=8, y_min=-5, y_max=5, z_min=-0.5, z_max=1;
  double leaf_size=0.03, epsilon=0.44;
  int min_points=10, min_cluster_size=1, max_cluster_size=500;
  void validate() const;
};
struct Box {
  Point center, size;
  unsigned int point_count;
};
struct Result {
  Cloud::Ptr filtered{new Cloud};
  std::vector<Box> boxes;
};
Result detect(const Cloud& input, const Config& config);
}  // namespace lidar_perception
