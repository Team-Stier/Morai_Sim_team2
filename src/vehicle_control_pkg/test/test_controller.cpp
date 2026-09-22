#include <gtest/gtest.h>
#include <cmath>
#include "controller.h"

namespace {
nav_msgs::Odometry odometry(double speed = 0.0) {
  nav_msgs::Odometry o;
  o.header.stamp = ros::Time(10.0);
  o.header.frame_id = "odom";
  o.child_frame_id = "base_link";
  o.pose.pose.orientation.w = 1.0;
  o.twist.twist.linear.x = speed;
  return o;
}
common_msgs_pkg::Trajectory trajectory(double speed = 2.0, double y = 0.0) {
  common_msgs_pkg::Trajectory t;
  t.header.stamp = ros::Time(10.0);
  t.header.frame_id = "odom";
  t.reset_id = 3;
  t.valid = true;
  t.valid_for = ros::Duration(0.5);
  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::Pose pose;
    pose.position.x = i;
    pose.position.y = y;
    pose.orientation.w = 1.0;
    t.poses.push_back(pose);
    t.speed_mps.push_back(speed);
    t.time_from_start.push_back(ros::Duration(i*0.1));
  }
  return t;
}
}

TEST(Controller, MapsSiSpeedIntoOriginalPiAndPreservesStamps) {
  vehicle_control::Controller controller{vehicle_control::Config{}};
  const auto o = controller.step(odometry(1.0), trajectory(2.0), ros::Time(10.1), 0.02);
  EXPECT_TRUE(o.command.valid);
  EXPECT_DOUBLE_EQ(o.command.accel, 0.08*3.6 + 0.02*3.6*0.02);
  EXPECT_DOUBLE_EQ(o.command.brake, 0.0);
  EXPECT_EQ(o.command.header.frame_id, "base_link");
  EXPECT_EQ(o.command.header.stamp, ros::Time(10.1));
  EXPECT_EQ(o.command.gear, common_msgs_pkg::ActuatorCommand::GEAR_DRIVE);
  EXPECT_EQ(o.status.odometry_stamp, ros::Time(10.0));
  EXPECT_EQ(o.status.trajectory_stamp, ros::Time(10.0));
  EXPECT_EQ(o.status.reset_id, 3U);
  EXPECT_EQ(o.status.mode, common_msgs_pkg::ControllerStatus::PURE_PURSUIT);
  EXPECT_DOUBLE_EQ(o.status.target_speed_mps, 2.0);
}

TEST(Controller, InterpolatesPlannerSpeedAndBrakesWhenAboveTarget) {
  vehicle_control::Controller controller{vehicle_control::Config{}};
  auto t = trajectory();
  t.speed_mps[0] = 1.0; t.speed_mps[1] = 3.0;
  auto o = controller.step(odometry(3.0), t, ros::Time(10.05), 0.02);
  EXPECT_NEAR(o.status.target_speed_mps, 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(o.command.accel, 0.0);
  EXPECT_NEAR(o.command.brake, 0.288, 1e-9);
}

TEST(Controller, PlannerStopCanHaveEmptyGeometry) {
  vehicle_control::Controller controller{vehicle_control::Config{}};
  common_msgs_pkg::Trajectory t;
  t.valid = true; t.stop_required = true;
  auto o = controller.step(odometry(), t, ros::Time(10.1), 0.02);
  EXPECT_TRUE(o.command.valid);
  EXPECT_TRUE(o.status.stop_required);
  EXPECT_DOUBLE_EQ(o.command.accel, 0.0);
  EXPECT_DOUBLE_EQ(o.command.brake, 0.35);
}

TEST(Controller, FrenetPreviewAcceleratesFromMeasuredInitialSpeed) {
  vehicle_control::Config config;
  config.reference_preview_sec=0.5;
  vehicle_control::Controller controller(config);
  auto t=trajectory();
  for (size_t i=0;i<t.speed_mps.size();++i) t.speed_mps[i]=2.0*t.time_from_start[i].toSec();
  auto out=controller.step(odometry(0.0),t,ros::Time(10.0),0.02);
  EXPECT_TRUE(out.command.valid);
  EXPECT_NEAR(out.status.target_speed_mps,1.0,1e-9);
  EXPECT_GT(out.command.accel,0.0);
}

TEST(Controller, ShortTerminalStopStillSendsValidBrake) {
  vehicle_control::Controller controller{vehicle_control::Config{}};
  auto t=trajectory();
  t.poses.resize(3);t.speed_mps={2.0,1.0,0.0};t.time_from_start.resize(3);
  auto out=controller.step(odometry(2.0),t,ros::Time(10.0),0.02);
  EXPECT_TRUE(out.command.valid);
  EXPECT_GT(out.command.brake,0.0);
  EXPECT_EQ(out.status.reason,"terminal_stop_approach");
}

TEST(Controller, LeftAndRightSteeringKeepOriginalSignAndStepLimit) {
  for (double y : {-0.5, 0.5}) {
    vehicle_control::Controller controller{vehicle_control::Config{}};
    double previous = 0.0;
    for (int i = 0; i < 20; ++i) {
      const auto o = controller.step(odometry(), trajectory(2.0, y), ros::Time(10.0+i*0.02), 0.02);
      EXPECT_TRUE(o.command.valid);
      EXPECT_LE(std::abs(o.command.steering_rad-previous), 0.0300001);
      previous = o.command.steering_rad;
    }
    EXPECT_GT(previous*y, 0.0);
  }
}

TEST(Controller, RotatedOdomPathUsesPoseYaw) {
  auto odom = odometry();
  odom.pose.pose.position.x = 100.0;
  odom.pose.pose.position.y = 200.0;
  odom.pose.pose.orientation.z = std::sqrt(0.5);
  odom.pose.pose.orientation.w = std::sqrt(0.5);
  auto t = trajectory();
  for (auto& pose : t.poses) {
    const double x = pose.position.x;
    pose.position.x = 100.0;
    pose.position.y = 200.0+x;
    pose.orientation = odom.pose.pose.orientation;
  }
  vehicle_control::Controller controller{vehicle_control::Config{}};
  for (int i = 0; i < 20; ++i) {
    auto o = controller.step(odom, t, ros::Time(10.0+i*0.02), 0.02);
    EXPECT_TRUE(o.command.valid);
    EXPECT_NEAR(o.command.steering_rad, 0.0, 1e-9);
  }
}

TEST(OriginalPi, StopResetsIntegral) {
  longitudinal_control::BoundedPiController pi(0.08, 0.02, 0.08, 10.0, 0.35);
  pi.calculate(20.0, 0.0, 0.1, false);
  EXPECT_DOUBLE_EQ(pi.calculate(0.0, 0.0, 0.1, true).second, 0.35);
  EXPECT_DOUBLE_EQ(pi.calculate(1.0, 0.0, 0.1, false).first, 0.082);
}

TEST(Controller, TunedAccelerationStillBrakesForLowerTrajectoryAndResetsOnStop) {
  vehicle_control::Config config;
  config.kp = 0.10;
  config.ki = 0.02;
  config.reference_preview_sec = 0.5;
  vehicle_control::Controller controller(config);
  auto acceleration = trajectory(5.0);
  for (size_t i = 0; i < acceleration.speed_mps.size(); ++i)
    acceleration.speed_mps[i] = 5.0 + 2.5 * acceleration.time_from_start[i].toSec();
  auto out = controller.step(odometry(5.0), acceleration, ros::Time(10.0), 0.02);
  EXPECT_TRUE(out.command.valid);
  EXPECT_NEAR(out.status.target_speed_mps, 6.25, 1e-9);
  EXPECT_GT(out.command.accel, 0.45);
  EXPECT_DOUBLE_EQ(out.command.brake, 0.0);
  out = controller.step(odometry(5.0), trajectory(4.0), ros::Time(10.0), 0.02);
  EXPECT_DOUBLE_EQ(out.command.accel, 0.0);
  EXPECT_NEAR(out.command.brake, 0.288, 1e-9);
  auto stop = trajectory(0.0);
  stop.stop_required = true;
  out = controller.step(odometry(0.0), stop, ros::Time(10.0), 0.02);
  EXPECT_TRUE(out.status.stop_required);
  EXPECT_DOUBLE_EQ(out.command.accel, 0.0);
  EXPECT_DOUBLE_EQ(out.command.brake, 0.35);
}
