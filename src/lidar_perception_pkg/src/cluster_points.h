#pragma once
#include <sensor_msgs/PointCloud2.h>
#include <vector>
namespace lidar_perception {
sensor_msgs::PointCloud2 clusterPoints(const sensor_msgs::PointCloud2& raw,
                                       const std::vector<int>& cluster_ids);
}
