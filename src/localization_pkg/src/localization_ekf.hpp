/*
 * localization_ekf.hpp
 * - 역할: GPS 위치 보정과 IMU dead reckoning을 결합하는 15-state error-state EKF이다.
 * - 외부 ROS 인터페이스는 ego_state_estimator_node.cpp가 소유한다.
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace localization_pkg {

struct FilterConfig {
  double minimum_yaw_initialization_distance_m = 1.0;
  double gps_innovation_gate_chi2 = 16.27;
  double max_gps_correction_m = 5.0;
  double max_gps_measurement_delay_sec = 0.2;
  double default_gps_position_stddev_m = 2.5;
  double accelerometer_noise_stddev_mps2 = 0.35;
  double gyroscope_noise_stddev_radps = 0.03;
  double accelerometer_bias_random_walk_stddev_mps2 = 0.01;
  double gyroscope_bias_random_walk_stddev_radps = 0.003;
  double gravity_mps2 = 9.80665;
};

struct ImuSample {
  double stamp_sec = 0.0;
  Eigen::Vector3d angular_velocity_radps = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration_mps2 = Eigen::Vector3d::Zero();
};

struct GpsSample {
  double stamp_sec = 0.0;
  Eigen::Vector3d position_m = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance_m2 = Eigen::Matrix3d::Identity();
};

enum class UpdateResult { kAccepted, kRejected, kWaitingForInitialization };

class LocalizationEkf {
 public:
  explicit LocalizationEkf(const FilterConfig& config);

  bool propagate(const ImuSample& sample);
  UpdateResult correct_gps(const GpsSample& sample);
  bool is_initialized() const;
  double state_stamp_sec() const;
  const Eigen::Vector3d& position_m() const;
  const Eigen::Vector3d& velocity_mps() const;
  const Eigen::Quaterniond& orientation() const;
  const Eigen::Matrix<double, 15, 15>& covariance() const;

 private:
  bool is_finite(const Eigen::Vector3d& value) const;
  static Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& value);
  static Eigen::Quaterniond small_angle_quaternion(const Eigen::Vector3d& angle_rad);
  void initialize_covariance();

  FilterConfig config_;
  bool initialized_ = false;
  bool has_origin_ = false;
  bool has_previous_gps_ = false;
  double state_stamp_sec_ = 0.0;
  double last_accepted_gps_stamp_sec_ = 0.0;
  Eigen::Vector3d origin_position_m_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d previous_gps_position_m_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_m_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_mps_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d accelerometer_bias_mps2_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyroscope_bias_radps_ = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 15, 15> covariance_ = Eigen::Matrix<double, 15, 15>::Identity();
};

}  // namespace localization_pkg
