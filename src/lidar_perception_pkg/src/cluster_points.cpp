#include "cluster_points.h"
#include <limits>
#include <stdexcept>
#include <algorithm>

namespace lidar_perception {
sensor_msgs::PointCloud2 clusterPoints(const sensor_msgs::PointCloud2& raw,
                                       const std::vector<int>& labels) {
  const uint64_t count=uint64_t(raw.width)*raw.height;
  if (raw.is_bigendian || labels.size()!=count || !raw.point_step ||
      uint64_t(raw.row_step)<uint64_t(raw.width)*raw.point_step ||
      raw.data.size()!=uint64_t(raw.row_step)*raw.height ||
      raw.point_step>std::numeric_limits<uint32_t>::max()-8)
    throw std::invalid_argument("invalid raw cluster point layout");
  for (const auto& field : raw.fields)
    if (field.name=="cluster_id" || field.name=="source_index")
      throw std::invalid_argument("raw field collides with cluster provenance");
  const size_t selected=std::count_if(labels.begin(),labels.end(),[](int id){return id>=0;});
  sensor_msgs::PointCloud2 result;
  result.header=raw.header; result.fields=raw.fields; result.height=1;
  result.is_bigendian=false; result.is_dense=raw.is_dense;
  result.point_step=raw.point_step+8;
  if (selected>std::numeric_limits<uint32_t>::max()/result.point_step ||
      count>std::numeric_limits<uint32_t>::max())
    throw std::invalid_argument("cluster point output exceeds PointCloud2 limits");
  for (int i=0;i<2;++i) {
    sensor_msgs::PointField field;
    field.name=i ? "source_index" : "cluster_id";
    field.offset=raw.point_step+i*4; field.datatype=field.UINT32; field.count=1;
    result.fields.push_back(field);
  }
  result.width=selected;result.row_step=result.width*result.point_step;
  result.data.resize(result.row_step);
  size_t destination=0;
  for (size_t i=0;i<labels.size();++i) {
    if (labels[i]<0) continue;
    const size_t source=(i/raw.width)*raw.row_step+(i%raw.width)*raw.point_step;
    std::copy_n(raw.data.begin()+source,raw.point_step,result.data.begin()+destination);
    for (int field=0;field<2;++field) {
      const uint32_t value=field ? i : labels[i];
      for (int byte=0;byte<4;++byte) result.data[destination+raw.point_step+field*4+byte]=(value>>(8*byte))&255;
    }
    destination+=result.point_step;
  }
  return result;
}
}
