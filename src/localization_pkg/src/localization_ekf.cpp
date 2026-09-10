/*
 * localization_ekf.cpp
 * - 역할: nominal state를 IMU로 전파하고 GPS position measurement로 보정한다.
 */
#include "localization_ekf.hpp"

#include <algorithm>
#include <cmath>

namespace localization_pkg {
namespace {
constexpr int kPositionIndex = 0;
constexpr int kVelocityIndex = 3;
constexpr int kOrientationIndex = 6;
constexpr int kAccelerometerBiasIndex = 9;
constexpr int kGyroscopeBiasIndex = 12;
constexpr int kStateSize = 15;
}

LocalizationEkf::LocalizationEkf(const FilterConfig& config) : config_(config) {
  initialize_covariance();
}

bool LocalizationEkf::propagate(const ImuSample& sample) {
  if (!std::isfinite(sample.stamp_sec) || !is_finite(sample.angular_velocity_radps) ||
      !is_finite(sample.linear_acceleration_mps2) || !initialized_) {
    return false;
  }
  if (state_stamp_sec_ == 0.0) {
    state_stamp_sec_ = sample.stamp_sec;
    return true;
  }
  const double delta_time_sec = sample.stamp_sec - state_stamp_sec_;
  if (!std::isfinite(delta_time_sec) || delta_time_sec <= 0.0 || delta_time_sec > 1.0) {
    return false;
  }

  const Eigen::Vector3d angular_rate_radps = sample.angular_velocity_radps - gyroscope_bias_radps_;
  const Eigen::Vector3d specific_force_mps2 =
      sample.linear_acceleration_mps2 - accelerometer_bias_mps2_;
  const Eigen::Quaterniond delta_orientation =
      small_angle_quaternion(angular_rate_radps * delta_time_sec);
  orientation_ = (orientation_ * delta_orientation).normalized();
  const Eigen::Vector3d gravity_mps2(0.0, 0.0, -config_.gravity_mps2);
  const Eigen::Vector3d world_acceleration_mps2 =
      orientation_ * specific_force_mps2 + gravity_mps2;
  position_m_ += velocity_mps_ * delta_time_sec +
                 0.5 * world_acceleration_mps2 * delta_time_sec * delta_time_sec;
  velocity_mps_ += world_acceleration_mps2 * delta_time_sec;

  Eigen::Matrix<double, kStateSize, kStateSize> transition =
      Eigen::Matrix<double, kStateSize, kStateSize>::Identity();
  transition.block<3, 3>(kPositionIndex, kVelocityIndex) =
      Eigen::Matrix3d::Identity() * delta_time_sec;
  transition.block<3, 3>(kVelocityIndex, kOrientationIndex) =
      -orientation_.toRotationMatrix() * skew_symmetric(specific_force_mps2) * delta_time_sec;
  transition.block<3, 3>(kVelocityIndex, kAccelerometerBiasIndex) =
      -orientation_.toRotationMatrix() * delta_time_sec;
  transition.block<3, 3>(kOrientationIndex, kGyroscopeBiasIndex) =
      -Eigen::Matrix3d::Identity() * delta_time_sec;

  Eigen::Matrix<double, kStateSize, kStateSize> process_noise =
      Eigen::Matrix<double, kStateSize, kStateSize>::Zero();
  const double accelerometer_variance =
      config_.accelerometer_noise_stddev_mps2 * config_.accelerometer_noise_stddev_mps2;
  const double gyroscope_variance =
      config_.gyroscope_noise_stddev_radps * config_.gyroscope_noise_stddev_radps;
  process_noise.block<3, 3>(kVelocityIndex, kVelocityIndex) =
      Eigen::Matrix3d::Identity() * accelerometer_variance * delta_time_sec;
  process_noise.block<3, 3>(kOrientationIndex, kOrientationIndex) =
      Eigen::Matrix3d::Identity() * gyroscope_variance * delta_time_sec;
  process_noise.block<3, 3>(kAccelerometerBiasIndex, kAccelerometerBiasIndex) =
      Eigen::Matrix3d::Identity() * config_.accelerometer_bias_random_walk_stddev_mps2 *
      config_.accelerometer_bias_random_walk_stddev_mps2 * delta_time_sec;
  process_noise.block<3, 3>(kGyroscopeBiasIndex, kGyroscopeBiasIndex) =
      Eigen::Matrix3d::Identity() * config_.gyroscope_bias_random_walk_stddev_radps *
      config_.gyroscope_bias_random_walk_stddev_radps * delta_time_sec;
  covariance_ = transition * covariance_ * transition.transpose() + process_noise;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  state_stamp_sec_ = sample.stamp_sec;
  return covariance_.allFinite();
}

UpdateResult LocalizationEkf::correct_gps(const GpsSample& sample) {
  if (!std::isfinite(sample.stamp_sec) || !is_finite(sample.position_m) ||
      !sample.covariance_m2.allFinite() || sample.covariance_m2.diagonal().minCoeff() <= 0.0) {
    return UpdateResult::kRejected;
  }
  if (!has_origin_) {
    origin_position_m_ = sample.position_m;
    previous_gps_position_m_.setZero();
    has_previous_gps_ = true;
    has_origin_ = true;
    return UpdateResult::kWaitingForInitialization;
  }

  const Eigen::Vector3d local_position_m = sample.position_m - origin_position_m_;
  if (!initialized_) {
    const Eigen::Vector3d delta_m = local_position_m - previous_gps_position_m_;
    if (delta_m.head<2>().norm() < config_.minimum_yaw_initialization_distance_m) {
      return UpdateResult::kWaitingForInitialization;
    }
    const double yaw_rad = std::atan2(delta_m.y(), delta_m.x());
    orientation_ = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ());
    position_m_ = local_position_m;
    previous_gps_position_m_ = local_position_m;
    state_stamp_sec_ = sample.stamp_sec;
    last_accepted_gps_stamp_sec_ = sample.stamp_sec;
    initialized_ = true;
    initialize_covariance();
    return UpdateResult::kAccepted;
  }
  if (sample.stamp_sec < last_accepted_gps_stamp_sec_ ||
      state_stamp_sec_ - sample.stamp_sec > config_.max_gps_measurement_delay_sec) {
    return UpdateResult::kRejected;
  }

  const Eigen::Vector3d innovation_m = local_position_m - position_m_;
  Eigen::Matrix<double, 3, kStateSize> measurement =
      Eigen::Matrix<double, 3, kStateSize>::Zero();
  measurement.block<3, 3>(0, kPositionIndex) = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d innovation_covariance =
      measurement * covariance_ * measurement.transpose() + sample.covariance_m2;
  const double mahalanobis_distance =
      innovation_m.transpose() * innovation_covariance.ldlt().solve(innovation_m);
  if (!std::isfinite(mahalanobis_distance) ||
      mahalanobis_distance > config_.gps_innovation_gate_chi2) {
    return UpdateResult::kRejected;
  }

  Eigen::Vector3d bounded_innovation_m = innovation_m;
  const double innovation_norm_m = bounded_innovation_m.norm();
  if (innovation_norm_m > config_.max_gps_correction_m) {
    bounded_innovation_m *= config_.max_gps_correction_m / innovation_norm_m;
  }
  const Eigen::Matrix<double, kStateSize, 3> gain =
      covariance_ * measurement.transpose() * innovation_covariance.inverse();
  const Eigen::Matrix<double, kStateSize, 1> correction = gain * bounded_innovation_m;
  position_m_ += correction.segment<3>(kPositionIndex);
  velocity_mps_ += correction.segment<3>(kVelocityIndex);
  orientation_ =
      (orientation_ * small_angle_quaternion(correction.segment<3>(kOrientationIndex)))
          .normalized();
  accelerometer_bias_mps2_ += correction.segment<3>(kAccelerometerBiasIndex);
  gyroscope_bias_radps_ += correction.segment<3>(kGyroscopeBiasIndex);
  const Eigen::Matrix<double, kStateSize, kStateSize> identity =
      Eigen::Matrix<double, kStateSize, kStateSize>::Identity();
  covariance_ = (identity - gain * measurement) * covariance_ *
                (identity - gain * measurement).transpose() +
                gain * sample.covariance_m2 * gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  previous_gps_position_m_ = local_position_m;
  last_accepted_gps_stamp_sec_ = sample.stamp_sec;
  return covariance_.allFinite() ? UpdateResult::kAccepted : UpdateResult::kRejected;
}

bool LocalizationEkf::is_initialized() const { return initialized_; }
double LocalizationEkf::state_stamp_sec() const { return state_stamp_sec_; }
const Eigen::Vector3d& LocalizationEkf::position_m() const { return position_m_; }
const Eigen::Vector3d& LocalizationEkf::velocity_mps() const { return velocity_mps_; }
const Eigen::Quaterniond& LocalizationEkf::orientation() const { return orientation_; }
const Eigen::Matrix<double, 15, 15>& LocalizationEkf::covariance() const { return covariance_; }

bool LocalizationEkf::is_finite(const Eigen::Vector3d& value) const { return value.allFinite(); }

Eigen::Matrix3d LocalizationEkf::skew_symmetric(const Eigen::Vector3d& value) {
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(), -value.y(), value.x(), 0.0;
  return result;
}

Eigen::Quaterniond LocalizationEkf::small_angle_quaternion(const Eigen::Vector3d& angle_rad) {
  const double angle_norm_rad = angle_rad.norm();
  if (angle_norm_rad < 1e-10) {
    return Eigen::Quaterniond(1.0, 0.5 * angle_rad.x(), 0.5 * angle_rad.y(), 0.5 * angle_rad.z());
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle_norm_rad, angle_rad / angle_norm_rad));
}

void LocalizationEkf::initialize_covariance() {
  covariance_.setZero();
  covariance_.block<3, 3>(kPositionIndex, kPositionIndex) = Eigen::Matrix3d::Identity() * 4.0;
  covariance_.block<3, 3>(kVelocityIndex, kVelocityIndex) = Eigen::Matrix3d::Identity() * 4.0;
  covariance_.block<3, 3>(kOrientationIndex, kOrientationIndex) = Eigen::Matrix3d::Identity() * 0.1;
  covariance_.block<3, 3>(kAccelerometerBiasIndex, kAccelerometerBiasIndex) =
      Eigen::Matrix3d::Identity() * 0.01;
  covariance_.block<3, 3>(kGyroscopeBiasIndex, kGyroscopeBiasIndex) =
      Eigen::Matrix3d::Identity() * 0.001;
}

}  // namespace localization_pkg
