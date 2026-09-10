// IMPORTED LEGACY REFERENCE ONLY: excluded from build/install/launch.
// Public names, types and frame semantics require central-contract migration.
/*
 * ego_state_estimator_node.cpp
 * - 역할: GPS/IMU ROS 입력을 LocalizationEkf에 전달하고 ego odometry와 품질 상태를
 *         발행한다.
 * 인터페이스
 * - sub /sensors/imu/data: sensor_msgs/Imu
 * - sub /sensors/gps/fix: sensor_msgs/NavSatFix
 * - pub /localization/ego/odometry: nav_msgs/Odometry
 * - pub /localization/ego/quality: common_msgs_pkg/LocalizationQuality
 */
#include <cmath>
#include <cstdint>
#include <memory>
#include <limits>
#include <string>

#include <common_msgs_pkg/LocalizationQuality.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "localization_ekf.hpp"

namespace localization_pkg {
namespace {
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kPi = 3.14159265358979323846;

bool is_finite(double value) { return std::isfinite(value); }
}

class EgoStateEstimatorNode {
 public:
  EgoStateEstimatorNode() : private_node_handle_("~") {
    FilterConfig config;
    private_node_handle_.param("minimum_yaw_initialization_distance_m",
                               config.minimum_yaw_initialization_distance_m,
                               config.minimum_yaw_initialization_distance_m);
    private_node_handle_.param("gps_innovation_gate_chi2", config.gps_innovation_gate_chi2,
                               config.gps_innovation_gate_chi2);
    private_node_handle_.param("max_gps_correction_m", config.max_gps_correction_m,
                               config.max_gps_correction_m);
    private_node_handle_.param("max_gps_measurement_delay_sec",
                               config.max_gps_measurement_delay_sec,
                               config.max_gps_measurement_delay_sec);
    private_node_handle_.param("default_gps_position_stddev_m",
                               config.default_gps_position_stddev_m,
                               config.default_gps_position_stddev_m);
    config_default_gps_position_stddev_m_ = config.default_gps_position_stddev_m;
    private_node_handle_.param("accelerometer_noise_stddev_mps2",
                               config.accelerometer_noise_stddev_mps2,
                               config.accelerometer_noise_stddev_mps2);
    private_node_handle_.param("gyroscope_noise_stddev_radps", config.gyroscope_noise_stddev_radps,
                               config.gyroscope_noise_stddev_radps);
    private_node_handle_.param("accelerometer_bias_random_walk_stddev_mps2",
                               config.accelerometer_bias_random_walk_stddev_mps2,
                               config.accelerometer_bias_random_walk_stddev_mps2);
    private_node_handle_.param("gyroscope_bias_random_walk_stddev_radps",
                               config.gyroscope_bias_random_walk_stddev_radps,
                               config.gyroscope_bias_random_walk_stddev_radps);
    private_node_handle_.param("gravity_mps2", config.gravity_mps2, config.gravity_mps2);
    private_node_handle_.param("imu_timeout_sec", imu_timeout_sec_, 0.2);
    private_node_handle_.param("gps_timeout_sec", gps_timeout_sec_, 0.5);
    private_node_handle_.param("max_dead_reckoning_sec", max_dead_reckoning_sec_, 15.0);
    double quality_publish_rate_hz = 10.0;
    private_node_handle_.param("quality_publish_rate_hz", quality_publish_rate_hz,
                               quality_publish_rate_hz);
    filter_ = std::make_unique<LocalizationEkf>(config);

    odometry_publisher_ = node_handle_.advertise<nav_msgs::Odometry>(
        "/localization/ego/odometry", 100);
    quality_publisher_ = node_handle_.advertise<common_msgs_pkg::LocalizationQuality>(
        "/localization/ego/quality", 10, true);
    imu_subscriber_ = node_handle_.subscribe("/sensors/imu/data", 50,
                                             &EgoStateEstimatorNode::handle_imu, this);
    gps_subscriber_ = node_handle_.subscribe("/sensors/gps/fix", 20,
                                             &EgoStateEstimatorNode::handle_gps, this);
    quality_timer_ = node_handle_.createTimer(
        ros::Duration(1.0 / quality_publish_rate_hz),
        &EgoStateEstimatorNode::publish_quality, this);
  }

 private:
  // 함수이름: handle_imu
  // 기능: 유효한 IMU를 EKF 예측에 적용하고 새 timestamp의 odometry를 발행한다.
  // 인자: IMU ROS 메시지
  // 반환값: 없음
  void handle_imu(const sensor_msgs::Imu::ConstPtr& message) {
    const double stamp_sec = message->header.stamp.toSec();
    ImuSample sample;
    sample.stamp_sec = stamp_sec;
    sample.angular_velocity_radps = Eigen::Vector3d(message->angular_velocity.x,
                                                     message->angular_velocity.y,
                                                     message->angular_velocity.z);
    sample.linear_acceleration_mps2 = Eigen::Vector3d(message->linear_acceleration.x,
                                                       message->linear_acceleration.y,
                                                       message->linear_acceleration.z);
    if (!filter_->propagate(sample)) {
      last_detail_ =
          "Rejected IMU: non-finite, out-of-order, invalid interval, or not initialized.";
      return;
    }
    last_imu_stamp_ = message->header.stamp;
    last_detail_ = "IMU prediction applied.";
    publish_odometry();
  }

  // 함수이름: handle_gps
  // 기능: WGS84 GPS를 첫 fix 기준 local ENU로 변환해 EKF 위치 보정에 적용한다.
  // 인자: GPS ROS 메시지
  // 반환값: 없음
  void handle_gps(const sensor_msgs::NavSatFix::ConstPtr& message) {
    if (message->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX ||
        !is_finite(message->latitude) || !is_finite(message->longitude) ||
        !is_finite(message->altitude) || message->header.stamp.isZero()) {
      ++rejected_gps_count_;
      gps_rejected_since_last_accept_ = true;
      last_detail_ = "Rejected GPS: no fix, non-finite value, or zero timestamp.";
      return;
    }
    const Eigen::Vector3d geodetic_position(message->latitude, message->longitude,
                                             message->altitude);
    if (!has_geodetic_origin_) {
      geodetic_origin_ = geodetic_position;
      has_geodetic_origin_ = true;
    }
    GpsSample sample;
    sample.stamp_sec = message->header.stamp.toSec();
    sample.position_m = convert_geodetic_to_local_enu(geodetic_position);
    sample.covariance_m2 = gps_covariance(*message);
    const UpdateResult result = filter_->correct_gps(sample);
    if (result == UpdateResult::kRejected) {
      ++rejected_gps_count_;
      gps_rejected_since_last_accept_ = true;
      last_detail_ = "Rejected GPS: timestamp or innovation gate failed.";
      return;
    }
    last_gps_stamp_ = message->header.stamp;
    gps_rejected_since_last_accept_ = false;
    last_detail_ = result == UpdateResult::kAccepted ? "GPS correction applied."
                                                      : "Waiting for GPS heading initialization.";
  }

  Eigen::Vector3d convert_geodetic_to_local_enu(const Eigen::Vector3d& geodetic_position) const {
    const double latitude_rad = geodetic_position.x() * kPi / 180.0;
    const double origin_latitude_rad = geodetic_origin_.x() * kPi / 180.0;
    const double longitude_delta_rad =
        (geodetic_position.y() - geodetic_origin_.y()) * kPi / 180.0;
    const double latitude_delta_rad = latitude_rad - origin_latitude_rad;
    return Eigen::Vector3d(kEarthRadiusM * std::cos(origin_latitude_rad) * longitude_delta_rad,
                           kEarthRadiusM * latitude_delta_rad,
                           geodetic_position.z() - geodetic_origin_.z());
  }

  Eigen::Matrix3d gps_covariance(const sensor_msgs::NavSatFix& message) const {
    Eigen::Matrix3d covariance;
    covariance << message.position_covariance[0], message.position_covariance[1],
        message.position_covariance[2], message.position_covariance[3],
        message.position_covariance[4], message.position_covariance[5],
        message.position_covariance[6], message.position_covariance[7],
        message.position_covariance[8];
    if (message.position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN ||
        !covariance.allFinite() || covariance.diagonal().minCoeff() <= 0.0) {
      covariance = Eigen::Matrix3d::Identity() *
                   config_default_gps_position_stddev_m_ * config_default_gps_position_stddev_m_;
    }
    return covariance;
  }

  // 함수이름: publish_odometry
  // 기능: 최신 IMU timestamp에 해당하는 EKF pose와 covariance를 발행한다.
  // 인자: 없음
  // 반환값: 없음
  void publish_odometry() {
    nav_msgs::Odometry message;
    message.header.stamp = ros::Time(filter_->state_stamp_sec());
    message.header.frame_id = "local_enu";
    message.child_frame_id = "base_link";
    const Eigen::Vector3d& position_m = filter_->position_m();
    const Eigen::Vector3d& velocity_mps = filter_->velocity_mps();
    const Eigen::Quaterniond& orientation = filter_->orientation();
    message.pose.pose.position.x = position_m.x();
    message.pose.pose.position.y = position_m.y();
    message.pose.pose.position.z = position_m.z();
    message.pose.pose.orientation.w = orientation.w();
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.twist.twist.linear.x = velocity_mps.x();
    message.twist.twist.linear.y = velocity_mps.y();
    message.twist.twist.linear.z = velocity_mps.z();
    const Eigen::Matrix<double, 15, 15>& covariance = filter_->covariance();
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        message.pose.covariance[row * 6 + column] = covariance(row, column);
        message.pose.covariance[(row + 3) * 6 + (column + 3)] = covariance(row + 6, column + 6);
      }
    }
    odometry_publisher_.publish(message);
  }

  // 함수이름: publish_quality
  // 기능: GPS/IMU freshness와 EKF lifecycle을 정기적으로 발행한다.
  // 인자: timer event
  // 반환값: 없음
  void publish_quality(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const double imu_age_sec = last_imu_stamp_.isZero()
                                   ? std::numeric_limits<double>::infinity()
                                   : (now - last_imu_stamp_).toSec();
    const double gps_age_sec = last_gps_stamp_.isZero()
                                   ? std::numeric_limits<double>::infinity()
                                   : (now - last_gps_stamp_).toSec();
    common_msgs_pkg::LocalizationQuality message;
    message.header.stamp = now;
    message.state_estimate_stamp = ros::Time(filter_->state_stamp_sec());
    message.position_covariance_trace_m2 = filter_->covariance().block<3, 3>(0, 0).trace();
    message.gps_age_sec = gps_age_sec;
    message.imu_age_sec = imu_age_sec;
    message.rejected_gps_count = rejected_gps_count_;
    if (!filter_->is_initialized()) {
      message.state = common_msgs_pkg::LocalizationQuality::INITIALIZING;
    } else if (imu_age_sec > imu_timeout_sec_ || !std::isfinite(imu_age_sec) ||
               gps_age_sec > max_dead_reckoning_sec_) {
      message.state = common_msgs_pkg::LocalizationQuality::INVALID;
    } else if (gps_age_sec > gps_timeout_sec_ || gps_rejected_since_last_accept_) {
      message.state = common_msgs_pkg::LocalizationQuality::DEGRADED;
    } else {
      message.state = common_msgs_pkg::LocalizationQuality::NOMINAL;
    }
    message.detail = last_detail_;
    quality_publisher_.publish(message);
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber gps_subscriber_;
  ros::Publisher odometry_publisher_;
  ros::Publisher quality_publisher_;
  ros::Timer quality_timer_;
  std::unique_ptr<LocalizationEkf> filter_;
  bool has_geodetic_origin_ = false;
  Eigen::Vector3d geodetic_origin_ = Eigen::Vector3d::Zero();
  ros::Time last_imu_stamp_;
  ros::Time last_gps_stamp_;
  double imu_timeout_sec_ = 0.2;
  double gps_timeout_sec_ = 0.5;
  double max_dead_reckoning_sec_ = 15.0;
  double config_default_gps_position_stddev_m_ = 2.5;
  uint32_t rejected_gps_count_ = 0;
  bool gps_rejected_since_last_accept_ = false;
  std::string last_detail_ = "Waiting for valid GPS and IMU data.";
};

}  // namespace localization_pkg

int main(int argc, char** argv) {
  ros::init(argc, argv, "ego_state_estimator");
  localization_pkg::EgoStateEstimatorNode node;
  ros::spin();
  return 0;
}
