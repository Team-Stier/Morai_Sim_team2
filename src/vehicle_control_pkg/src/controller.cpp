#include "controller.h"

#include <algorithm>
#include <cmath>

namespace vehicle_control {
namespace {
double yaw(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

// Producer supplies equal-sized arrays, zero initial time and increasing times.
double referenceSpeed(const common_msgs_pkg::Trajectory& trajectory, const ros::Time& now) {
  const double elapsed = (now-trajectory.header.stamp).toSec();
  const auto& times = trajectory.time_from_start;
  if (elapsed <= times.front().toSec()) return trajectory.speed_mps.front();
  if (elapsed >= times.back().toSec()) return trajectory.speed_mps.back();
  size_t i = 1;
  while (times[i].toSec() < elapsed) ++i;
  const double fraction = (elapsed-times[i-1].toSec()) / (times[i]-times[i-1]).toSec();
  return trajectory.speed_mps[i-1] + fraction*(trajectory.speed_mps[i]-trajectory.speed_mps[i-1]);
}
}  // namespace

Controller::Controller(const Config& c) : pp_(c.pp), stanley_(c.stanley),
    supervisor_(c.supervisor), speed_(c.kp, c.ki, c.brake_kp, c.integral_limit, c.stop_brake) {}

Output Controller::base(const ros::Time& now) const {
  Output o;
  o.command.header.stamp = now;
  o.command.header.frame_id = "base_link";
  o.command.gear = common_msgs_pkg::ActuatorCommand::GEAR_DRIVE;
  o.status.header.stamp = now;
  return o;
}

Output Controller::stop(const ros::Time& now, double dt, double speed_mps, const std::string& reason) {
  auto o = base(now);
  const hybrid_path_tracking::ControllerCandidate absent{false, 0, 0, 0, 0, false, ""};
  o.command.steering_rad = supervisor_.select(absent, absent,
      {hybrid_path_tracking::PathSource::GLOBAL, now.toSec(), false});
  const auto pedals = speed_.calculate(0.0, speed_mps*3.6, dt, true);
  o.command.brake = pedals.second;
  o.status.stop_required = true;
  o.status.mode = common_msgs_pkg::ControllerStatus::STOP;
  o.status.reason = reason;
  return o;
}

Output Controller::step(const nav_msgs::Odometry& odom, const common_msgs_pkg::Trajectory& traj,
                        const ros::Time& now, double dt) {
  if (traj.stop_required) {
    auto o = stop(now, dt, odom.twist.twist.linear.x, "planned_stop");
    o.command.valid = true;
    o.status.ready = o.status.command_valid = true;
    o.status.reset_id = traj.reset_id;
    o.status.odometry_stamp = odom.header.stamp;
    o.status.trajectory_stamp = traj.header.stamp;
    o.status.speed_error_mps = -odom.twist.twist.linear.x;
    return o;
  }
  hybrid_path_tracking::VehicleState2D state{odom.pose.pose.position.x, odom.pose.pose.position.y,
      yaw(odom.pose.pose.orientation), odom.twist.twist.linear.x * 3.6};
  hybrid_path_tracking::Path2D path{{}, 1.0, 0};
  for (const auto& pose : traj.poses) path.points.push_back({pose.position.x, pose.position.y});
  const auto pp = pp_.calculate(state, path);
  const auto stanley = stanley_.calculate(state, path);
  auto o = base(now);
  o.command.steering_rad = supervisor_.select(pp, stanley,
      {hybrid_path_tracking::PathSource::GLOBAL, now.toSec(), true});
  const auto mode = supervisor_.mode();
  if (mode == hybrid_path_tracking::HybridMode::STOP) {
    // The imported supervisor already rejected both candidates; propagate its stop.
    o.command.brake = speed_.calculate(0.0, state.speed_kph, dt, true).second;
    o.status.stop_required = true;
    o.status.reason = "upstream_controller_stop";
    return o;
  }
  if (mode == hybrid_path_tracking::HybridMode::STANLEY_GLOBAL || !pp.valid) uses_stanley_ = true;
  if (mode == hybrid_path_tracking::HybridMode::PP_GLOBAL || !stanley.valid) uses_stanley_ = false;
  const auto& chosen = uses_stanley_ ? stanley : pp;
  const double target = referenceSpeed(traj, now);
  const auto pedals = speed_.calculate(target*3.6, state.speed_kph, dt, target == 0.0);
  o.command.accel = pedals.first;
  o.command.brake = pedals.second;
  o.command.valid = true;
  o.status.ready = o.status.command_valid = true;
  o.status.stop_required = target == 0.0;
  o.status.mode = mode == hybrid_path_tracking::HybridMode::DEGRADED ? common_msgs_pkg::ControllerStatus::DEGRADED :
      uses_stanley_ ? common_msgs_pkg::ControllerStatus::STANLEY : common_msgs_pkg::ControllerStatus::PURE_PURSUIT;
  o.status.reset_id = traj.reset_id;
  o.status.odometry_stamp = odom.header.stamp;
  o.status.trajectory_stamp = traj.header.stamp;
  o.status.target_speed_mps = target;
  o.status.speed_error_mps = target-odom.twist.twist.linear.x;
  o.status.cross_track_error_m = chosen.cross_track_error_m;
  o.status.heading_error_rad = chosen.heading_error_rad;
  o.status.saturated = chosen.saturated;
  o.status.reason = target == 0.0 ? "planned_stop" : "tracking";
  return o;
}
}  // namespace vehicle_control
