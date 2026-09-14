#include "horizontalization.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lidar_perception {
namespace {
bool valid(const Eigen::Quaterniond& q) {
  return q.coeffs().allFinite() && std::isfinite(q.norm()) && q.norm()>1e-9;
}
}
Eigen::Matrix3d levelRotation(const Eigen::Quaterniond& attitude,
                              const Eigen::Quaterniond& mount) {
  if (!valid(attitude) || !valid(mount))
    throw std::invalid_argument("invalid leveling quaternion");
  const Eigen::Matrix3d body=attitude.normalized().toRotationMatrix();
  // Heading is undefined near vertical. Reject rather than picking an arbitrary yaw.
  if (std::hypot(body(0,0), body(1,0))<1e-6)
    throw std::invalid_argument("leveling attitude has undefined heading");
  const double yaw=std::atan2(body(1,0),body(0,0));
  return Eigen::AngleAxisd(-yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()
      *body*mount.normalized().toRotationMatrix();
}
Cloud rotateCloud(const Cloud& input, const Eigen::Matrix3d& rotation) {
  Cloud result=input;
  for (auto& p : result) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const Eigen::Vector3d v=rotation*Eigen::Vector3d(p.x,p.y,p.z);
    p.x=v.x(); p.y=v.y(); p.z=v.z();
  }
  return result;
}
Box boxInSensorFrame(const Box& b, const Eigen::Matrix3d& rotation) {
  const Eigen::Vector3d center=rotation.transpose()*Eigen::Vector3d(b.center.x,b.center.y,b.center.z);
  // abs(R^-1)*extent encloses all eight inverse-rotated corners.
  const Eigen::Vector3d size=rotation.transpose().cwiseAbs()*Eigen::Vector3d(b.size.x,b.size.y,b.size.z);
  Box result=b;
  result.center=Point(center.x(),center.y(),center.z());
  result.size=Point(size.x(),size.y(),size.z());
  return result;
}
bool AttitudeHistory::insert(uint64_t stamp, const Eigen::Quaterniond& q, double history_sec) {
  if (!stamp || !valid(q) || !std::isfinite(history_sec) || history_sec<=0 ||
      (!samples_.empty() && stamp<=samples_.back().stamp)) return false;
  samples_.push_back({stamp,q.normalized()});
  while (samples_.size()>2 && (stamp-samples_.front().stamp)*1e-9>history_sec) samples_.pop_front();
  // Bound memory even if a producer emits unreasonable timestamp increments.
  while (samples_.size()>1000) samples_.pop_front();
  return true;
}
bool AttitudeHistory::interpolate(uint64_t stamp, double gap, Eigen::Quaterniond& result) const {
  if (!std::isfinite(gap) || gap<=0) return false;
  const auto after=std::lower_bound(samples_.begin(),samples_.end(),stamp,
      [](const Sample& s,uint64_t t){return s.stamp<t;});
  if (after==samples_.end()) return false;
  if (after->stamp==stamp) {result=after->orientation;return true;}
  if (after==samples_.begin()) return false;
  const auto before=std::prev(after);
  if ((after->stamp-before->stamp)*1e-9>gap) return false;
  const double fraction=double(stamp-before->stamp)/double(after->stamp-before->stamp);
  result=before->orientation.slerp(fraction,after->orientation).normalized();
  return true;
}
}  // namespace lidar_perception
