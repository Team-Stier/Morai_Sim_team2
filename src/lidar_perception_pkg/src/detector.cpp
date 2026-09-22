#include "detector.h"
#include "dbscan.h"
#include <pcl/common/common.h>
#include <array>
#include <map>
#include <limits>
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
  result.raw_cluster_ids.assign(input.size(),-1);
  struct Voxel { Eigen::Vector3d sum=Eigen::Vector3d::Zero(); std::vector<size_t> indices; };
  // Explicit provenance avoids a dense PCL leaf layout (hundreds of MB for this
  // ROI) and avoids assigning boundary returns to a nearest, unrelated centroid.
  std::map<std::array<long long,3>,Voxel> voxels;
  const float inverse_leaf=1.0f/static_cast<float>(config.leaf_size);
  for (size_t i=0;i<input.size();++i) {
    const auto& p=input[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (p.x>=config.x_min && p.x<=config.x_max &&
        p.y>=config.y_min && p.y<=config.y_max &&
        p.z>=config.z_min && p.z<=config.z_max) {
      std::array<long long,3> key;
      // PCL's global leaf coordinates, sorted with x fastest, then y, then z.
      for (int axis=0;axis<3;++axis) {
        const double cell=std::floor(p.getVector3fMap()[axis]*inverse_leaf);
        if (!std::isfinite(cell) || cell<=static_cast<double>(std::numeric_limits<long long>::min()) ||
            cell>=static_cast<double>(std::numeric_limits<long long>::max()))
          throw std::runtime_error("voxel coordinates overflow");
        key[2-axis]=static_cast<long long>(cell);
      }
      auto& voxel=voxels[key]; voxel.sum+=p.getVector3fMap().cast<double>(); voxel.indices.push_back(i);
    }
  }
  std::vector<std::vector<size_t>> provenance;
  provenance.reserve(voxels.size()); result.filtered->reserve(voxels.size());
  for (const auto& pair : voxels) {
    const auto& voxel=pair.second;
    const Eigen::Vector3d mean=voxel.sum/static_cast<double>(voxel.indices.size());
    result.filtered->push_back(Point(mean.x(),mean.y(),mean.z()));
    provenance.push_back(voxel.indices);
  }
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
    for (int index : cluster.indices) {
      points.push_back((*result.filtered)[index]);
      for (size_t raw_index : provenance[index]) result.raw_cluster_ids[raw_index]=result.boxes.size();
    }
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
