#pragma once

#include <common_msgs_pkg/ActuatorCommand.h>
#include <common_msgs_pkg/ControllerStatus.h>
#include <common_msgs_pkg/Trajectory.h>
#include <nav_msgs/Odometry.h>
#include "hybrid_path_tracking/hybrid_supervisor.hpp"
#include "hybrid_path_tracking/pure_pursuit.hpp"
#include "hybrid_path_tracking/stanley.hpp"
#include "bounded_pi.hpp"

namespace vehicle_control {
struct Config {
  hybrid_path_tracking::PurePursuitConfig pp;
  hybrid_path_tracking::StanleyConfig stanley;
  hybrid_path_tracking::SupervisorConfig supervisor;
  // Original PI units: km/h, not m/s.
  double kp{0.08}, ki{0.02}, brake_kp{0.08};
  double integral_limit{10.0}, stop_brake{0.35};
  double reference_preview_sec{0.0};
};

struct Output {
  common_msgs_pkg::ActuatorCommand command;
  common_msgs_pkg::ControllerStatus status;
};

class Controller {
 public:
  explicit Controller(const Config& config);
  Output step(const nav_msgs::Odometry& odometry,
              const common_msgs_pkg::Trajectory& trajectory,
              const ros::Time& now, double dt);
  Output stop(const ros::Time& now, double dt, double speed_mps, const std::string& reason);
 private:
  hybrid_path_tracking::PurePursuit pp_;
  hybrid_path_tracking::Stanley stanley_;
  hybrid_path_tracking::HybridSupervisor supervisor_;
  longitudinal_control::BoundedPiController speed_;
  double reference_preview_sec_;
  bool uses_stanley_{false};
  Output base(const ros::Time& now) const;
};
}  // namespace vehicle_control
