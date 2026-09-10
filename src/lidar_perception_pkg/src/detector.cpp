#include "detector.h"
#include "dbscan.h"
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <stdexcept>

namespace lidar_perception {
void Config::validate() const {
  for (double v : {x_min,x_max,y_min,y_max,z_min,z_max,leaf_size,epsilon})
    if (!std::isfinite(v)) throw std::invalid_argument("nonfinite detector parameter");
  if (x_min>=x_max || y_min>=y_max || z_min>=z_max || leaf_size<=0 ||
      epsilon<=0 || min_points<1 || min_cluster_size<1 ||
      max_cluster_size<min_cluster_size)
    throw std::invalid_argument("invalid ROI/voxel/DBSCAN parameter range");
}
Result detect(const Cloud& input, const Config& config) {
  config.validate();
  Result result;
  Cloud::Ptr roi(new Cloud);
  for (const auto& p : input) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (p.x>=config.x_min && p.x<=config.x_max &&
        p.y>=config.y_min && p.y<=config.y_max &&
        p.z>=config.z_min && p.z<=config.z_max) roi->push_back(p);
  }
  if (roi->empty()) return result;
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(roi);
  voxel.setLeafSize(config.leaf_size, config.leaf_size, config.leaf_size);
  voxel.filter(*result.filtered);
  if (result.filtered->empty()) return result;
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  tree->setInputCloud(result.filtered);
  DBSCANKdtreeCluster<Point> dbscan;
  dbscan.setInputCloud(result.filtered);
  dbscan.setSearchMethod(tree);
  dbscan.setCorePointMinPts(config.min_points);
  dbscan.setClusterTolerance(config.epsilon);
  dbscan.setMinClusterSize(config.min_cluster_size);
  dbscan.setMaxClusterSize(config.max_cluster_size);
  std::vector<pcl::PointIndices> indices;
  dbscan.extract(indices);
  for (const auto& cluster : indices) {
    Cloud points;
    for (int index : cluster.indices) points.push_back((*result.filtered)[index]);
    Point low, high;
    pcl::getMinMax3D(points, low, high);
    Box box;
    box.center.x=(low.x+high.x)/2; box.center.y=(low.y+high.y)/2; box.center.z=(low.z+high.z)/2;
    box.size.x=high.x-low.x; box.size.y=high.y-low.y; box.size.z=high.z-low.z;
    box.point_count=points.size();
    result.boxes.push_back(box);
  }
  return result;
}
}  // namespace lidar_perception
