#include "controller.h"

#include <common_msgs_pkg/ComponentStatus.h>
#include <common_msgs_pkg/LocalizationStatus.h>
#include <ros/ros.h>

namespace vehicle_control {
Config loadConfig(ros::NodeHandle& nh) {
  Config c;
  nh.param("vehicle/wheelbase_m", c.pp.wheelbase_m, c.pp.wheelbase_m);
  nh.param("vehicle/max_steering_rad", c.pp.max_steering_rad, c.pp.max_steering_rad);
  c.stanley.wheelbase_m = c.pp.wheelbase_m;
  c.stanley.max_steering_rad = c.pp.max_steering_rad;
  nh.param("pure_pursuit/min_lookahead_m", c.pp.min_lookahead_m, c.pp.min_lookahead_m);
  nh.param("pure_pursuit/max_lookahead_m", c.pp.max_lookahead_m, c.pp.max_lookahead_m);
  nh.param("pure_pursuit/lookahead_time_sec", c.pp.lookahead_time_sec, c.pp.lookahead_time_sec);
  nh.param("pure_pursuit/min_forward_path_m", c.pp.min_forward_path_m, c.pp.min_forward_path_m);
  nh.param("stanley/heading_gain", c.stanley.heading_gain, c.stanley.heading_gain);
  nh.param("stanley/cross_track_gain", c.stanley.cross_track_gain, c.stanley.cross_track_gain);
  nh.param("stanley/softening_mps", c.stanley.softening_mps, c.stanley.softening_mps);
  nh.param("stanley/min_path_length_m", c.stanley.min_path_length_m, c.stanley.min_path_length_m);
  nh.param("supervisor/curvature_enter_stanley", c.supervisor.curvature_enter_stanley, c.supervisor.curvature_enter_stanley);
  nh.param("supervisor/curvature_exit_stanley", c.supervisor.curvature_exit_stanley, c.supervisor.curvature_exit_stanley);
  nh.param("supervisor/cross_track_enter_stanley_m", c.supervisor.cross_track_enter_stanley_m, c.supervisor.cross_track_enter_stanley_m);
  nh.param("supervisor/cross_track_exit_stanley_m", c.supervisor.cross_track_exit_stanley_m, c.supervisor.cross_track_exit_stanley_m);
  nh.param("supervisor/confirmation_samples", c.supervisor.confirmation_samples, c.supervisor.confirmation_samples);
  nh.param("supervisor/minimum_dwell_sec", c.supervisor.minimum_dwell_sec, c.supervisor.minimum_dwell_sec);
  nh.param("supervisor/blend_duration_sec", c.supervisor.blend_duration_sec, c.supervisor.blend_duration_sec);
  nh.param("supervisor/max_step_rad", c.supervisor.max_step_rad, c.supervisor.max_step_rad);
  nh.param("longitudinal/kp", c.kp, c.kp);
  nh.param("longitudinal/ki", c.ki, c.ki);
  nh.param("longitudinal/brake_kp", c.brake_kp, c.brake_kp);
  nh.param("longitudinal/integrator_limit", c.integral_limit, c.integral_limit);
  nh.param("longitudinal/stop_brake", c.stop_brake, c.stop_brake);
  return c;
}

class Node {
 public:
  Node() : private_("~"), controller_(loadConfig(private_)) {
    private_.param("development_global_path_only", global_path_only_, false);
    double control_rate = 50.0, status_rate = 10.0;
    int input_queue = 2, command_queue = 2, status_queue = 1;
    bool latched = true;
    private_.getParam("contract/runtime/control_rate_hz", control_rate);
    private_.getParam("contract/runtime/status_rate_hz", status_rate);
    private_.getParam("contract/runtime/input_queue_size", input_queue);
    private_.getParam("contract/runtime/command_queue_size", command_queue);
    private_.getParam("contract/runtime/status_queue_size", status_queue);
    private_.getParam("contract/runtime/status_latched", latched);
    command_pub_ = nh_.advertise<common_msgs_pkg::ActuatorCommand>("/molit/control/nominal_command", command_queue);
    status_pub_ = nh_.advertise<common_msgs_pkg::ControllerStatus>("/molit/control/status", status_queue, latched);
    odom_sub_ = nh_.subscribe("/molit/localization/local/odometry", input_queue, &Node::odometry, this);
    loc_sub_ = nh_.subscribe("/molit/localization/status", input_queue, &Node::localization, this);
    traj_sub_ = nh_.subscribe("/molit/planning/trajectory", input_queue, &Node::trajectory, this);
    plan_sub_ = nh_.subscribe("/molit/planning/status", input_queue, &Node::planning, this);
    control_timer_ = nh_.createTimer(ros::Duration(1.0/control_rate), &Node::update, this);
    status_timer_ = nh_.createTimer(ros::Duration(1.0/status_rate), &Node::publishStatus, this);
  }
 private:
  void odometry(const nav_msgs::Odometry::ConstPtr& m) { odometry_ = m; }
  void localization(const common_msgs_pkg::LocalizationStatus::ConstPtr& m) { localization_ = m; }
  void trajectory(const common_msgs_pkg::Trajectory::ConstPtr& m) { trajectory_ = m; }
  void planning(const common_msgs_pkg::ComponentStatus::ConstPtr& m) { planning_ = m; }
  void update(const ros::TimerEvent& event) {
    const auto now = ros::Time::now();
    const double dt = (event.current_real-event.last_real).toSec();
    const double speed = odometry_ ? odometry_->twist.twist.linear.x : 0.0;
    Output o;
    if (!odometry_ || !localization_ || !trajectory_ || !planning_) {
      o = controller_.stop(now, dt, speed, "waiting_for_inputs");
    } else if (!localization_->local_odometry_valid || (localization_->stop_required && !global_path_only_) ||
               !planning_->ready || planning_->stop_required || !trajectory_->valid) {
      o = controller_.stop(now, dt, speed, "upstream_stop_required");
    } else {
      o = controller_.step(*odometry_, *trajectory_, now, dt);
    }
    // Preserve producer stamps as telemetry; only command/status use generation time.
    if (odometry_) o.status.odometry_stamp = odometry_->header.stamp;
    if (trajectory_) o.status.trajectory_stamp = trajectory_->header.stamp;
    if (localization_) o.status.reset_id = localization_->reset_id;
    status_ = o.status;
    command_pub_.publish(o.command);
  }
  void publishStatus(const ros::TimerEvent&) { status_pub_.publish(status_); }
  ros::NodeHandle nh_, private_;
  Controller controller_;
  bool global_path_only_{false};
  ros::Publisher command_pub_, status_pub_;
  ros::Subscriber odom_sub_, loc_sub_, traj_sub_, plan_sub_;
  ros::Timer control_timer_, status_timer_;
  nav_msgs::Odometry::ConstPtr odometry_;
  common_msgs_pkg::LocalizationStatus::ConstPtr localization_;
  common_msgs_pkg::Trajectory::ConstPtr trajectory_;
  common_msgs_pkg::ComponentStatus::ConstPtr planning_;
  common_msgs_pkg::ControllerStatus status_;
};
}  // namespace vehicle_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "vehicle_controller_node");
  vehicle_control::Node node;
  ros::spin();
  return 0;
}
