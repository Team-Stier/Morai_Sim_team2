/*
localization_supervisor_node.cpp
- 역할: Localization 입력과 Global EKF 출력을 감시하고 안전한
        odometry만 relay한다.
- 주요 클래스: LocalizationSupervisorNode
인터페이스
- pub topics.output_odometry: nav_msgs/Odometry
- pub topics.status: diagnostic_msgs/DiagnosticArray
- sub topics.input_imu: sensor_msgs/Imu
- sub topics.input_vehicle_twist: geometry_msgs/TwistWithCovarianceStamped
- sub topics.gps_state: std_msgs/String
- sub topics.gps_map_pose: geometry_msgs/PoseWithCovarianceStamped
- sub topics.global_filtered_odometry: nav_msgs/Odometry
*/
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include "localization_pkg/freshness.hpp"
#include "localization_pkg/localization_health.hpp"

namespace localization_pkg {
namespace {

// 함수이름: requireParameter
// 기능: 필수 ROS parameter를 읽고 누락된 경우 node 시작을 중단한다.
// 인자: node - ROS NodeHandle, name - parameter 이름
// 반환값: 요청한 타입의 parameter 값
template <typename T>
T requireParameter(const ros::NodeHandle& node, const std::string& name) {
  T value;
  if (!node.getParam(name, value)) {
    throw std::runtime_error("missing required parameter: " + name);
  }
  return value;
}

// 함수이름: requireAbsoluteName
// 기능: topic 이름이 '/'로 시작하는 전역 ROS 이름인지 검사한다.
// 인자: value - 검사할 ROS 이름, name - 오류에 표시할 parameter 이름
// 반환값: 없음
void requireAbsoluteName(const std::string& value, const std::string& name) {
  if (value.empty() || value.front() != '/') {
    throw std::runtime_error(name + " must be an absolute ROS name");
  }
}

// 함수이름: requireFinitePositive
// 기능: 설정값이 유한하고 0보다 큰지 검사한다.
// 인자: value - 검사할 값, name - 오류에 표시할 parameter 이름
// 반환값: 없음
void requireFinitePositive(double value, const std::string& name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(name + " must be finite and positive");
  }
}

// 함수이름: requireFiniteNonnegative
// 기능: 설정값이 유한하고 음수가 아닌지 검사한다.
// 인자: value - 검사할 값, name - 오류에 표시할 parameter 이름
// 반환값: 없음
void requireFiniteNonnegative(double value, const std::string& name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::runtime_error(name + " must be finite and nonnegative");
  }
}

// 함수이름: gpsStateIsWhitelisted
// 기능: GPS projector가 발행할 수 있는 상태 문자열인지 검사한다.
// 인자: state - 검사할 GPS 상태 문자열
// 반환값: 허용된 상태이면 true, 아니면 false
bool gpsStateIsWhitelisted(const std::string& state) {
  return state == "WAITING_FOR_FIX" || state == "TRACKING" ||
         state == "DEGRADED" || state == "RELOCALIZING";
}

// 함수이름: diagnosticValue
// 기능: diagnostics 배열에 넣을 key/value 한 쌍을 만든다.
// 인자: key, value
// 반환값: 입력 문자열을 담은 diagnostic_msgs::KeyValue
diagnostic_msgs::KeyValue diagnosticValue(const std::string& key,
                                          const std::string& value) {
  diagnostic_msgs::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

// 함수이름: formatAge
// 기능: 수신 여부와 두 시각을 사람이 읽을 수 있는 초 단위 age 문자열로
//       변환한다.
// 인자: received, sample_time_sec, now_sec
// 반환값: age 문자열 또는 "unavailable"
std::string formatAge(bool received, double sample_time_sec,
                      double now_sec) {
  if (!received || !std::isfinite(sample_time_sec) ||
      !std::isfinite(now_sec) || sample_time_sec <= 0.0 || now_sec <= 0.0) {
    return "unavailable";
  }
  const double age_sec = now_sec - sample_time_sec;
  if (!std::isfinite(age_sec)) {
    return "unavailable";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << age_sec;
  return stream.str();
}

struct StampedStreamStatus {
  // 하나의 입력 stream에 대해 수신 여부, 마지막 검증 결과와 거부 이유를 보관한다.
  // 마지막 승인 ROS timestamp와 monotonic 수신 시간은 freshness 판정에 사용된다.
  // 거부 sample은 received/valid만 바꾸며 마지막 승인 시간은 갱신하지 않는다.
  bool received = false;
  bool valid = false;
  bool have_accepted_stamp = false;
  double accepted_stamp_sec = 0.0;
  double accepted_receipt_steady_sec = 0.0;
  std::string rejection_reason;
};

}  // namespace

class LocalizationSupervisorNode {
  // 입력: 원본 IMU·차량 속도, GPS 상태·map pose와 Global EKF odometry.
  // 처리: frame, timestamp, 유한값, covariance, heartbeat와 GPS anchor 반영을 검증한다.
  // 출력: 안전 조건을 만족할 때만 Global EKF odometry를 최종 topic에 그대로 relay하고
  //       모든 평가 주기마다 diagnostic_msgs/DiagnosticArray를 발행한다.
  // 상태: 각 stream의 마지막 승인 시간, 최신 Global EKF 값과 anchor 확인 여부를 기억한다.
  // 책임 경계: 센서 값을 EKF에 대신 전달하거나 새로운 위치 추정값을 계산하지 않는다.
 public:
  // 함수이름: LocalizationSupervisorNode
  // 기능: 설정을 읽고 publisher, subscriber와 health 평가 timer를 연결한다.
  // 인자: 없음
  // 반환값: 없음
  LocalizationSupervisorNode() : node_() {
    loadConfiguration();

    output_publisher_ =
        node_.advertise<nav_msgs::Odometry>(output_odometry_topic_, 10, false);
    diagnostics_publisher_ = node_.advertise<diagnostic_msgs::DiagnosticArray>(
        status_topic_, 10, false);
    imu_subscriber_ = node_.subscribe(input_imu_topic_, 20,
                                      &LocalizationSupervisorNode::imuCallback,
                                      this);
    twist_subscriber_ = node_.subscribe(
        input_vehicle_twist_topic_, 20,
        &LocalizationSupervisorNode::twistCallback, this);
    gps_state_subscriber_ = node_.subscribe(
        gps_state_topic_, 10, &LocalizationSupervisorNode::gpsStateCallback,
        this);
    gps_anchor_subscriber_ = node_.subscribe(
        gps_map_pose_topic_, 20,
        &LocalizationSupervisorNode::gpsAnchorCallback, this);
    global_odometry_subscriber_ = node_.subscribe(
        global_odometry_topic_, 20,
        &LocalizationSupervisorNode::globalOdometryCallback, this);
    evaluation_timer_ = node_.createSteadyTimer(
        ros::WallDuration(1.0 / publish_rate_hz_),
        &LocalizationSupervisorNode::evaluationCallback, this);
  }

 private:
  // 함수이름: loadConfiguration
  // 기능: topic, frame, 주기, timeout과 validation 설정을 읽고 검증한다.
  // 인자: 없음
  // 반환값: 없음
  void loadConfiguration() {
    input_imu_topic_ =
        requireParameter<std::string>(node_, "topics/input_imu");
    input_vehicle_twist_topic_ = requireParameter<std::string>(
        node_, "topics/input_vehicle_twist");
    gps_state_topic_ =
        requireParameter<std::string>(node_, "topics/gps_state");
    gps_map_pose_topic_ =
        requireParameter<std::string>(node_, "topics/gps_map_pose");
    global_odometry_topic_ = requireParameter<std::string>(
        node_, "topics/global_filtered_odometry");
    output_odometry_topic_ = requireParameter<std::string>(
        node_, "topics/output_odometry");
    status_topic_ = requireParameter<std::string>(node_, "topics/status");

    map_frame_ = requireParameter<std::string>(node_, "frames/map");
    base_link_frame_ =
        requireParameter<std::string>(node_, "frames/base_link");
    imu_frame_ =
        requireParameter<std::string>(node_, "sensor_frames/imu");
    vehicle_twist_frame_ = requireParameter<std::string>(
        node_, "sensor_frames/vehicle_twist");

    publish_rate_hz_ =
        requireParameter<double>(node_, "runtime/publish_rate_hz");
    sensor_timeout_sec_ =
        requireParameter<double>(node_, "runtime/sensor_timeout_sec");
    gps_message_max_age_sec_ = requireParameter<double>(
        node_, "runtime/gps_message_max_age_sec");
    gps_state_timeout_sec_ = requireParameter<double>(
        node_, "runtime/gps_state_timeout_sec");
    global_odometry_timeout_sec_ = requireParameter<double>(
        node_, "runtime/global_odometry_timeout_sec");
    max_future_stamp_sec_ =
        requireParameter<double>(node_, "runtime/max_future_stamp_sec");
    min_quaternion_norm_ =
        requireParameter<double>(node_, "validation/min_quaternion_norm");
    quaternion_unit_norm_tolerance_ = requireParameter<double>(
        node_, "validation/quaternion_unit_norm_tolerance");
    global_anchor_max_error_m_ = requireParameter<double>(
        node_, "validation/global_anchor_max_error_m");

    requireAbsoluteName(input_imu_topic_, "topics/input_imu");
    requireAbsoluteName(input_vehicle_twist_topic_,
                        "topics/input_vehicle_twist");
    requireAbsoluteName(gps_state_topic_, "topics/gps_state");
    requireAbsoluteName(gps_map_pose_topic_, "topics/gps_map_pose");
    requireAbsoluteName(global_odometry_topic_,
                        "topics/global_filtered_odometry");
    requireAbsoluteName(output_odometry_topic_, "topics/output_odometry");
    requireAbsoluteName(status_topic_, "topics/status");
    if (map_frame_.empty() || base_link_frame_.empty() || imu_frame_.empty() ||
        vehicle_twist_frame_.empty()) {
      throw std::runtime_error("configured frame names must be non-empty");
    }
    requireFinitePositive(publish_rate_hz_, "runtime/publish_rate_hz");
    requireFinitePositive(sensor_timeout_sec_, "runtime/sensor_timeout_sec");
    requireFinitePositive(gps_message_max_age_sec_,
                          "runtime/gps_message_max_age_sec");
    requireFinitePositive(gps_state_timeout_sec_,
                          "runtime/gps_state_timeout_sec");
    requireFinitePositive(global_odometry_timeout_sec_,
                          "runtime/global_odometry_timeout_sec");
    requireFiniteNonnegative(max_future_stamp_sec_,
                             "runtime/max_future_stamp_sec");
    requireFinitePositive(min_quaternion_norm_,
                          "validation/min_quaternion_norm");
    requireFinitePositive(quaternion_unit_norm_tolerance_,
                          "validation/quaternion_unit_norm_tolerance");
    requireFinitePositive(global_anchor_max_error_m_,
                          "validation/global_anchor_max_error_m");
    if (quaternion_unit_norm_tolerance_ >= 1.0) {
      throw std::runtime_error(
          "validation/quaternion_unit_norm_tolerance must be less than one");
    }
  }

  // 함수이름: validateStamp
  // 기능: timestamp의 유효성, 단조 증가와 freshness를 검사한다.
  // 인자: stamp_sec, last_stamp_sec, have_last_stamp, now_sec, max_age_sec, prefix
  // 반환값: 성공 시 빈 문자열, 실패 시 diagnostics 거부 이유
  std::string validateStamp(double stamp_sec, double last_stamp_sec,
                            bool have_last_stamp, double now_sec,
                            double timeout_sec,
                            const std::string& stream_name) const {
    if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
      return stream_name + "_stamp_invalid";
    }
    if (have_last_stamp && stamp_sec <= last_stamp_sec) {
      return stream_name + "_stamp_not_monotonic";
    }
    if (!timestampIsFresh(stamp_sec, now_sec, timeout_sec,
                          max_future_stamp_sec_)) {
      return stream_name + "_stamp_not_fresh";
    }
    return std::string();
  }

  // 함수이름: validateQuaternion
  // 기능: quaternion을 검사하고 stream 이름이 포함된 거부 이유로 변환한다.
  // 인자: x, y, z, w, prefix
  // 반환값: 성공 시 빈 문자열, 실패 시 diagnostics 거부 이유
  std::string validateQuaternion(
      const geometry_msgs::Quaternion& quaternion,
      const std::string& stream_name) const {
    switch (validateUnitQuaternion(
        quaternion.x, quaternion.y, quaternion.z, quaternion.w,
        min_quaternion_norm_, quaternion_unit_norm_tolerance_)) {
      case QuaternionValidity::VALID:
        return std::string();
      case QuaternionValidity::NONFINITE:
        return stream_name + "_quaternion_nonfinite";
      case QuaternionValidity::ZERO:
        return stream_name + "_zero_quaternion";
      case QuaternionValidity::NOT_UNIT:
        return stream_name + "_quaternion_not_unit";
      case QuaternionValidity::INVALID_ARGUMENT:
        return stream_name + "_quaternion_validation_invalid";
    }
    return stream_name + "_quaternion_validation_invalid";
  }

  // 함수이름: covarianceReason
  // 기능: covariance 검증 결과를 stream별 거부 이유로 변환한다.
  // 인자: validity, prefix
  // 반환값: 성공 시 빈 문자열, 실패 시 diagnostics 거부 이유
  std::string covarianceReason(CovarianceValidity validity,
                               const std::string& stream_name) const {
    switch (validity) {
      case CovarianceValidity::VALID:
        return std::string();
      case CovarianceValidity::UNAVAILABLE:
        return stream_name + "_unavailable";
      case CovarianceValidity::NONFINITE:
        return stream_name + "_nonfinite";
      case CovarianceValidity::NEGATIVE_DIAGONAL:
        return stream_name + "_negative_diagonal";
      case CovarianceValidity::INVALID_ARGUMENT:
        return stream_name + "_validation_invalid";
    }
    return stream_name + "_validation_invalid";
  }

  // 함수이름: rejectSample
  // 기능: 마지막 승인 시간을 보존하면서 sample 거부 상태와 이유를
  //       기록한다.
  // 인자: status - 갱신할 stream 상태, reason - 거부 이유
  // 반환값: 없음
  void rejectSample(StampedStreamStatus* status,
                    const std::string& reason) {
    status->received = true;
    status->valid = false;
    status->rejection_reason = reason;
  }

  // 함수이름: acceptSample
  // 기능: 승인한 sample의 ROS timestamp와 monotonic 수신 시간을 저장한다.
  // 인자: status, stamp_sec, receipt_steady_sec
  // 반환값: 없음
  void acceptSample(StampedStreamStatus* status, double stamp_sec,
                    double receipt_steady_sec) {
    status->received = true;
    status->valid = true;
    status->have_accepted_stamp = true;
    status->accepted_stamp_sec = stamp_sec;
    status->accepted_receipt_steady_sec = receipt_steady_sec;
    status->rejection_reason.clear();
  }

  // 함수이름: imuCallback
  // 기능: IMU의 timestamp, frame, orientation, yaw rate와 covariance를 검증한다.
  // 인자: message - 수신한 sensor_msgs::Imu
  // 반환값: 없음
  void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    const double stamp_sec = message->header.stamp.toSec();
    const double ros_now_sec = ros::Time::now().toSec();
    const double steady_now_sec = ros::SteadyTime::now().toSec();
    std::string reason = validateStamp(
        stamp_sec, imu_status_.accepted_stamp_sec,
        imu_status_.have_accepted_stamp, ros_now_sec, sensor_timeout_sec_,
        "imu");
    if (reason.empty() && message->header.frame_id != imu_frame_) {
      reason = "imu_frame_mismatch";
    }
    if (reason.empty()) {
      reason = validateQuaternion(message->orientation, "imu");
    }
    if (reason.empty() && !std::isfinite(message->angular_velocity.z)) {
      reason = "imu_angular_velocity_z_nonfinite";
    }
    if (reason.empty()) {
      reason = covarianceReason(
          validateCovarianceMatrix(message->orientation_covariance.data(), 3,
                                   true),
          "imu_orientation_covariance");
    }
    if (reason.empty()) {
      reason = covarianceReason(
          validateCovarianceMatrix(
              message->angular_velocity_covariance.data(), 3, true),
          "imu_angular_velocity_covariance");
    }
    if (!reason.empty()) {
      rejectSample(&imu_status_, reason);
      return;
    }
    acceptSample(&imu_status_, stamp_sec, steady_now_sec);
  }

  // 함수이름: twistCallback
  // 기능: 차량 속도의 timestamp, frame, 유한값과 covariance를 검증한다.
  // 인자: message - 수신한 geometry_msgs::TwistWithCovarianceStamped
  // 반환값: 없음
  void twistCallback(
      const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& message) {
    const double stamp_sec = message->header.stamp.toSec();
    const double ros_now_sec = ros::Time::now().toSec();
    const double steady_now_sec = ros::SteadyTime::now().toSec();
    std::string reason = validateStamp(
        stamp_sec, twist_status_.accepted_stamp_sec,
        twist_status_.have_accepted_stamp, ros_now_sec, sensor_timeout_sec_,
        "vehicle_twist");
    if (reason.empty() &&
        message->header.frame_id != vehicle_twist_frame_) {
      reason = "vehicle_twist_frame_mismatch";
    }
    if (reason.empty() &&
        (!std::isfinite(message->twist.twist.linear.x) ||
         !std::isfinite(message->twist.twist.linear.y))) {
      reason = "vehicle_twist_nonfinite";
    }
    if (reason.empty()) {
      reason = covarianceReason(
          validateCovarianceMatrix(message->twist.covariance.data(), 6,
                                   false),
          "vehicle_twist_covariance");
    }
    if (!reason.empty()) {
      rejectSample(&twist_status_, reason);
      return;
    }
    acceptSample(&twist_status_, stamp_sec, steady_now_sec);
  }

  // 함수이름: gpsStateCallback
  // 기능: GPS 상태 heartbeat를 검증하고 재위치 추정 진입 시 anchor
  //       상태를 초기화한다.
  // 인자: message - 수신한 std_msgs::String
  // 반환값: 없음
  void gpsStateCallback(const std_msgs::String::ConstPtr& message) {
    const bool entering_relocalization =
        message->data == "RELOCALIZING" &&
        (!gps_state_received_ || gps_state_ != "RELOCALIZING");
    if (entering_relocalization) {
      global_anchor_readiness_.clear();
      global_anchor_status_ = StampedStreamStatus();
    }
    gps_state_received_ = true;
    gps_state_receipt_steady_sec_ = ros::SteadyTime::now().toSec();
    gps_state_ = message->data;
    gps_state_valid_ = gpsStateIsWhitelisted(gps_state_);
    gps_state_rejection_reason_ =
        gps_state_valid_ ? std::string() : "gps_state_not_whitelisted";
  }

  // 함수이름: gpsAnchorCallback
  // 기능: GPS map pose를 검증하고 Global EKF 반영 확인용 anchor로 저장한다.
  // 인자: message - 수신한 geometry_msgs::PoseWithCovarianceStamped
  // 반환값: 없음
  void gpsAnchorCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message) {
    if (global_anchor_readiness_.confirmed()) {
      return;
    }
    const double stamp_sec = message->header.stamp.toSec();
    const double ros_now_sec = ros::Time::now().toSec();
    const double steady_now_sec = ros::SteadyTime::now().toSec();
    std::string reason = validateStamp(
        stamp_sec, global_anchor_status_.accepted_stamp_sec,
        global_anchor_status_.have_accepted_stamp, ros_now_sec,
        gps_message_max_age_sec_, "global_anchor");
    if (reason.empty() && message->header.frame_id != map_frame_) {
      reason = "global_anchor_frame_mismatch";
    }
    if (reason.empty() &&
        (!std::isfinite(message->pose.pose.position.x) ||
         !std::isfinite(message->pose.pose.position.y) ||
         !std::isfinite(message->pose.pose.position.z))) {
      reason = "global_anchor_nonfinite_pose";
    }
    if (reason.empty()) {
      reason = validateQuaternion(message->pose.pose.orientation,
                                  "global_anchor");
    }
    if (reason.empty()) {
      reason = covarianceReason(
          validateCovarianceMatrix(message->pose.covariance.data(), 6, false),
          "global_anchor_covariance");
    }
    if (!reason.empty()) {
      rejectSample(&global_anchor_status_, reason);
      return;
    }
    acceptSample(&global_anchor_status_, stamp_sec, steady_now_sec);
    if (!global_anchor_readiness_.updateAnchor(
            stamp_sec, steady_now_sec, message->pose.pose.position.x,
            message->pose.pose.position.y)) {
      rejectSample(&global_anchor_status_, "global_anchor_update_failed");
    }
  }

  // 함수이름: globalOdometryCallback
  // 기능: Global EKF odometry를 검증해 relay 후보와 anchor 확인 상태를 갱신한다.
  // 인자: message - 수신한 nav_msgs::Odometry
  // 반환값: 없음
  void globalOdometryCallback(const nav_msgs::Odometry::ConstPtr& message) {
    const double stamp_sec = message->header.stamp.toSec();
    const double ros_now_sec = ros::Time::now().toSec();
    const double steady_now_sec = ros::SteadyTime::now().toSec();
    std::string reason = validateStamp(
        stamp_sec, global_odometry_status_.accepted_stamp_sec,
        global_odometry_status_.have_accepted_stamp, ros_now_sec,
        global_odometry_timeout_sec_, "global_odometry");
    if (reason.empty() &&
        (message->header.frame_id != map_frame_ ||
         message->child_frame_id != base_link_frame_)) {
      reason = "global_odometry_frame_mismatch";
    }
    if (reason.empty() &&
        (!std::isfinite(message->pose.pose.position.x) ||
         !std::isfinite(message->pose.pose.position.y) ||
         !std::isfinite(message->pose.pose.position.z))) {
      reason = "global_odometry_nonfinite_pose";
    }
    if (reason.empty()) {
      reason = validateQuaternion(message->pose.pose.orientation,
                                  "global_odometry");
    }
    if (reason.empty() &&
        (!std::isfinite(message->twist.twist.linear.x) ||
         !std::isfinite(message->twist.twist.linear.y) ||
         !std::isfinite(message->twist.twist.linear.z) ||
         !std::isfinite(message->twist.twist.angular.x) ||
         !std::isfinite(message->twist.twist.angular.y) ||
         !std::isfinite(message->twist.twist.angular.z))) {
      reason = "global_odometry_nonfinite_twist";
    }
    if (reason.empty()) {
      reason = covarianceReason(
          validateCovarianceMatrix(message->pose.covariance.data(), 6,
                                   false),
          "global_odometry_pose_covariance");
    }
    if (reason.empty()) {
      reason = covarianceReason(
          validateCovarianceMatrix(message->twist.covariance.data(), 6,
                                   false),
          "global_odometry_twist_covariance");
    }
    if (!reason.empty()) {
      rejectSample(&global_odometry_status_, reason);
      return;
    }
    acceptSample(&global_odometry_status_, stamp_sec, steady_now_sec);
    latest_global_odometry_ = *message;
    if (streamIsFresh(global_anchor_status_, ros_now_sec, steady_now_sec,
                      gps_message_max_age_sec_)) {
      global_anchor_readiness_.tryConfirm(
          gps_state_received_ && gps_state_valid_ && gps_state_ == "TRACKING",
          stamp_sec, steady_now_sec, message->pose.pose.position.x,
          message->pose.pose.position.y, global_anchor_max_error_m_);
    }
  }

  // 함수이름: streamIsFresh
  // 기능: stream의 마지막 승인 timestamp와 수신 시간이 모두 timeout
  //       안인지 확인한다.
  // 인자: status, ros_now_sec, steady_now_sec, max_age_sec
  // 반환값: 두 시간 기준을 모두 만족하면 true, 아니면 false
  bool streamIsFresh(const StampedStreamStatus& status, double ros_now_sec,
                     double steady_now_sec, double timeout_sec) const {
    return status.valid && status.have_accepted_stamp &&
           timestampIsFresh(status.accepted_stamp_sec, ros_now_sec,
                            timeout_sec, max_future_stamp_sec_) &&
           receiptIsFresh(status.accepted_receipt_steady_sec, steady_now_sec,
                          timeout_sec);
  }

  // 함수이름: makeHealthInput
  // 기능: callback이 저장한 stream 상태를 HealthInput으로 구성한다.
  // 인자: ros_now_sec, steady_now_sec
  // 반환값: 현재 Localization 입력 상태를 담은 HealthInput
  HealthInput makeHealthInput(double ros_now_sec, double steady_now_sec) const {
    HealthInput input;
    input.imu_received = imu_status_.received;
    input.imu_valid = imu_status_.valid;
    input.imu_fresh = streamIsFresh(imu_status_, ros_now_sec, steady_now_sec,
                                    sensor_timeout_sec_);
    input.imu_rejection_reason = imu_status_.rejection_reason;

    input.twist_received = twist_status_.received;
    input.twist_valid = twist_status_.valid;
    input.twist_fresh = streamIsFresh(twist_status_, ros_now_sec, steady_now_sec,
                                      sensor_timeout_sec_);
    input.twist_rejection_reason = twist_status_.rejection_reason;

    input.gps_state_received = gps_state_received_;
    input.gps_state_valid = gps_state_valid_;
    input.gps_state_fresh = gps_state_received_ && receiptIsFresh(
        gps_state_receipt_steady_sec_, steady_now_sec,
        gps_state_timeout_sec_);
    input.gps_state = gps_state_;
    input.gps_state_rejection_reason = gps_state_rejection_reason_;

    input.global_odometry_received = global_odometry_status_.received;
    input.global_odometry_valid = global_odometry_status_.valid;
    input.global_odometry_fresh = streamIsFresh(
        global_odometry_status_, ros_now_sec, steady_now_sec,
        global_odometry_timeout_sec_);
    input.global_odometry_rejection_reason =
        global_odometry_status_.rejection_reason;
    input.global_anchor_received = global_anchor_status_.received;
    input.global_anchor_valid = global_anchor_status_.valid;
    input.global_anchor_fresh = streamIsFresh(
        global_anchor_status_, ros_now_sec, steady_now_sec,
        gps_message_max_age_sec_);
    input.global_anchor_confirmed = global_anchor_readiness_.confirmed();
    input.global_anchor_rejection_reason =
        global_anchor_status_.rejection_reason;
    return input;
  }

  // 함수이름: evaluationCallback
  // 기능: 현재 health를 평가해 허용 상태에서만 Global EKF odometry를 relay한다.
  // 인자: ros::SteadyTimerEvent - 사용하지 않음
  // 반환값: 없음
  void evaluationCallback(const ros::SteadyTimerEvent&) {
    const ros::Time ros_now = ros::Time::now();
    const double ros_now_sec = ros_now.toSec();
    const double steady_now_sec = ros::SteadyTime::now().toSec();
    const HealthDecision decision =
        evaluateHealth(makeHealthInput(ros_now_sec, steady_now_sec));

    if (decision.publish_output) {
      output_publisher_.publish(latest_global_odometry_);
    }
    publishDiagnostics(decision, ros_now, ros_now_sec, steady_now_sec);
  }

  // 함수이름: publishDiagnostics
  // 기능: health 상태와 stream age 및 anchor 상태를 DiagnosticArray로 발행한다.
  // 인자: decision, ros_stamp, ros_now_sec, steady_now_sec
  // 반환값: 없음
  void publishDiagnostics(const HealthDecision& decision,
                          const ros::Time& ros_now, double ros_now_sec,
                          double steady_now_sec) {
    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.header.stamp = ros_now;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "molit_localization_health";
    status.hardware_id = "localization";
    status.message = decision.rejection_reason;
    if (decision.state == LocalizationState::FAULT) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    } else if (decision.state == LocalizationState::TRACKING) {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    }

    status.values.push_back(diagnosticValue(
        "state", localizationStateName(decision.state)));
    status.values.push_back(
        diagnosticValue("rejection_reason", decision.rejection_reason));
    status.values.push_back(diagnosticValue(
        "gps_state", gps_state_received_ ? gps_state_ : "MISSING"));
    status.values.push_back(diagnosticValue(
        "imu_age_sec", formatAge(imu_status_.have_accepted_stamp,
                                  imu_status_.accepted_receipt_steady_sec,
                                  steady_now_sec)));
    status.values.push_back(diagnosticValue(
        "imu_stamp_age_sec", formatAge(imu_status_.have_accepted_stamp,
                                        imu_status_.accepted_stamp_sec,
                                        ros_now_sec)));
    status.values.push_back(diagnosticValue(
        "twist_age_sec", formatAge(twist_status_.have_accepted_stamp,
                                    twist_status_.accepted_receipt_steady_sec,
                                    steady_now_sec)));
    status.values.push_back(diagnosticValue(
        "twist_stamp_age_sec", formatAge(twist_status_.have_accepted_stamp,
                                          twist_status_.accepted_stamp_sec,
                                          ros_now_sec)));
    status.values.push_back(diagnosticValue(
        "gps_state_age_sec", formatAge(gps_state_received_,
                                        gps_state_receipt_steady_sec_,
                                        steady_now_sec)));
    status.values.push_back(diagnosticValue(
        "global_anchor_confirmed",
        global_anchor_readiness_.confirmed() ? "true" : "false"));
    status.values.push_back(diagnosticValue(
        "global_anchor_age_sec",
        formatAge(global_anchor_status_.have_accepted_stamp,
                  global_anchor_status_.accepted_receipt_steady_sec,
                  steady_now_sec)));
    status.values.push_back(diagnosticValue(
        "filter_age_sec",
        formatAge(global_odometry_status_.have_accepted_stamp,
                  global_odometry_status_.accepted_receipt_steady_sec,
                  steady_now_sec)));
    status.values.push_back(diagnosticValue(
        "filter_stamp_age_sec",
        formatAge(global_odometry_status_.have_accepted_stamp,
                  global_odometry_status_.accepted_stamp_sec, ros_now_sec)));
    diagnostics.status.push_back(status);
    diagnostics_publisher_.publish(diagnostics);
  }

  ros::NodeHandle node_;
  ros::Publisher output_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber twist_subscriber_;
  ros::Subscriber gps_state_subscriber_;
  ros::Subscriber gps_anchor_subscriber_;
  ros::Subscriber global_odometry_subscriber_;
  ros::SteadyTimer evaluation_timer_;

  std::string input_imu_topic_;
  std::string input_vehicle_twist_topic_;
  std::string gps_state_topic_;
  std::string gps_map_pose_topic_;
  std::string global_odometry_topic_;
  std::string output_odometry_topic_;
  std::string status_topic_;
  std::string map_frame_;
  std::string base_link_frame_;
  std::string imu_frame_;
  std::string vehicle_twist_frame_;
  double publish_rate_hz_ = 0.0;
  double sensor_timeout_sec_ = 0.0;
  double gps_message_max_age_sec_ = 0.0;
  double gps_state_timeout_sec_ = 0.0;
  double global_odometry_timeout_sec_ = 0.0;
  double max_future_stamp_sec_ = 0.0;
  double min_quaternion_norm_ = 0.0;
  double quaternion_unit_norm_tolerance_ = 0.0;
  double global_anchor_max_error_m_ = 0.0;

  StampedStreamStatus imu_status_;
  StampedStreamStatus twist_status_;
  StampedStreamStatus global_odometry_status_;
  StampedStreamStatus global_anchor_status_;
  GlobalAnchorReadiness global_anchor_readiness_;
  bool gps_state_received_ = false;
  bool gps_state_valid_ = false;
  std::string gps_state_;
  std::string gps_state_rejection_reason_;
  double gps_state_receipt_steady_sec_ = 0.0;
  nav_msgs::Odometry latest_global_odometry_;
};

}  // namespace localization_pkg

// 함수이름: main
// 기능: Localization supervisor ROS node를 초기화하고 callback 처리를 시작한다.
// 인자: argc, argv - ROS command-line 인자
// 반환값: 정상 종료는 0, 초기화 실패는 1
int main(int argc, char** argv) {
  ros::init(argc, argv, "molit_localization_supervisor");
  try {
    localization_pkg::LocalizationSupervisorNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL_STREAM("localization supervisor startup failed: "
                     << exception.what());
    return 1;
  }
  return 0;
}

