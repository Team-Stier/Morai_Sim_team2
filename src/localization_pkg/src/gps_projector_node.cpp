/*
gps_projector_node.cpp
- 역할: GPS를 map 좌표로 투영하고 GPS gate 및 Global EKF SetPose 절차를 관리한다.
- 주요 클래스: GpsProjectorNode
인터페이스
- pub topics.gps_map_pose: geometry_msgs/PoseWithCovarianceStamped
- pub topics.gps_state: std_msgs/String
- sub topics.input_gps_fix: sensor_msgs/NavSatFix
- sub topics.global_filtered_odometry: nav_msgs/Odometry
- client services.global_set_pose: robot_localization/SetPose
*/
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <robot_localization/SetPose.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/String.h>

#include "localization_pkg/freshness.hpp"
#include "localization_pkg/gps_gate.hpp"
#include "localization_pkg/gps_projection.hpp"

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

// 함수이름: finiteQuaternion
// 기능: quaternion 네 성분이 모두 유한한지 검사한다.
// 인자: quaternion - 검사할 quaternion
// 반환값: 모든 성분이 유한하면 true, 아니면 false
bool finiteQuaternion(const geometry_msgs::Quaternion& quaternion) {
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
}

// 함수이름: normalizeQuaternion
// 기능: 입력 quaternion을 단위 길이로 정규화한다.
// 인자: input, min_norm, output - 정규화 결과를 받을 포인터
// 반환값: 정규화에 성공하면 true, 입력이나 포인터가 유효하지 않으면 false
bool normalizeQuaternion(const geometry_msgs::Quaternion& input,
                         double min_norm,
                         geometry_msgs::Quaternion* output) {
  if (output == nullptr || !finiteQuaternion(input) ||
      !std::isfinite(min_norm) || min_norm <= 0.0) {
    return false;
  }
  const double norm = std::sqrt(input.x * input.x + input.y * input.y +
                                input.z * input.z + input.w * input.w);
  if (!std::isfinite(norm) || norm < min_norm) {
    return false;
  }
  output->x = input.x / norm;
  output->y = input.y / norm;
  output->z = input.z / norm;
  output->w = input.w / norm;
  return true;
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

// 함수이름: requireFinitePositive
// 기능: 설정값이 유한하고 0보다 큰지 검사한다.
// 인자: value - 검사할 값, name - 오류에 표시할 parameter 이름
// 반환값: 없음
void requireFinitePositive(double value, const std::string& name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(name + " must be finite and positive");
  }
}

// 함수이름: requireAbsoluteName
// 기능: topic 또는 service 이름이 전역 ROS 이름 형식인지 검사한다.
// 인자: value - 검사할 ROS 이름, name - 오류에 표시할 parameter 이름
// 반환값: 없음
void requireAbsoluteName(const std::string& value, const std::string& name) {
  if (value.empty() || value.front() != '/') {
    throw std::runtime_error(name + " must be an absolute ROS name");
  }
}

struct ResetCallResult {
  // 분리된 SetPose service 호출 thread가 ROS callback thread에 완료 결과를 전달한다.
  // node 객체를 직접 참조하지 않고 atomic 값만 공유해 비동기 호출 수명을 분리한다.
  // done은 완료 여부, success는 service 결과, completion_elapsed_sec은 소요 시간이다.
  // 함수이름: ResetCallResult
  // 기능: 비동기 SetPose 결과 공유 상태를 미완료 상태로 초기화한다.
  // 인자: 없음
  // 반환값: 없음
  ResetCallResult()
      : done(false), success(false), completion_elapsed_sec(0.0) {}

  std::atomic<bool> done;
  std::atomic<bool> success;
  std::atomic<double> completion_elapsed_sec;
};

}  // namespace

class GpsProjectorNode {
  // 입력: sensor_msgs/NavSatFix와 Global EKF의 최신 map odometry prediction.
  // 처리: GPS 검증, UTM 투영, innovation gate, 복귀 안정성 확인과 SetPose를 수행한다.
  // 출력: 검증된 PoseWithCovarianceStamped와 GPS gate 상태 heartbeat를 발행한다.
  // 상태: 마지막 fix·prediction 시간, GPS 수신 단계와 진행 중인 reset을 기억한다.
  // 책임 경계: IMU·차량 속도를 융합하거나 최종 Global Route Manager 출력을 만들지 않는다.
 public:
  // 함수이름: GpsProjectorNode
  // 기능: 설정과 core 객체를 준비하고 ROS 인터페이스와 timer를 연결한다.
  // 인자: 없음
  // 반환값: 없음
  GpsProjectorNode()
      : node_(),
        have_prediction_(false),
        have_accepted_fix_(false),
        have_candidate_fix_(false),
        have_last_fix_stamp_(false),
        have_last_odometry_stamp_(false),
        reset_call_inflight_(false),
        reset_call_timeout_reported_(false) {
    loadConfiguration();

    projector_.reset(new GpsProjection(
        utm_zone_, north_hemisphere_, origin_easting_m_, origin_northing_m_,
        map_z_));
    gate_.reset(new GpsGate(max_innovation_m_, stable_return_radius_m_,
                            static_cast<std::size_t>(required_fixes_)));

    pose_publisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        gps_pose_topic_, 10, false);
    state_publisher_ = node_.advertise<std_msgs::String>(gps_state_topic_, 1,
                                                         true);
    fix_subscriber_ = node_.subscribe(gps_fix_topic_, 20,
                                      &GpsProjectorNode::fixCallback, this);
    odometry_subscriber_ = node_.subscribe(
        global_odometry_topic_, 20, &GpsProjectorNode::odometryCallback, this);
    set_pose_client_ = node_.serviceClient<robot_localization::SetPose>(
        set_pose_service_, false);
    timeout_timer_ = node_.createSteadyTimer(
        ros::WallDuration(timeout_check_period_sec_),
        &GpsProjectorNode::timeoutCallback, this);
    publishState(true);
  }

 private:
  // 함수이름: loadConfiguration
  // 기능: topic, frame, 투영, timeout, covariance와 재수신 설정을 읽고 검증한다.
  // 인자: 없음
  // 반환값: 없음
  void loadConfiguration() {
    gps_fix_topic_ = requireParameter<std::string>(node_,
                                                    "topics/input_gps_fix");
    gps_pose_topic_ = requireParameter<std::string>(node_,
                                                    "topics/gps_map_pose");
    gps_state_topic_ = requireParameter<std::string>(node_,
                                                     "topics/gps_state");
    global_odometry_topic_ = requireParameter<std::string>(
        node_, "topics/global_filtered_odometry");
    set_pose_service_ = requireParameter<std::string>(
        node_, "services/global_set_pose");
    map_frame_ = requireParameter<std::string>(node_, "frames/map");
    base_link_frame_ = requireParameter<std::string>(node_,
                                                     "frames/base_link");

    utm_zone_ = requireParameter<int>(node_, "projection/utm_zone");
    north_hemisphere_ = requireParameter<bool>(
        node_, "projection/north_hemisphere");
    origin_easting_m_ = requireParameter<double>(
        node_, "projection/origin_easting_m");
    origin_northing_m_ = requireParameter<double>(
        node_, "projection/origin_northing_m");
    map_z_ = requireParameter<double>(node_,
                                      "projection/origin_altitude_m");

    gps_timeout_sec_ = requireParameter<double>(node_,
                                                 "runtime/gps_timeout_sec");
    gps_message_max_age_sec_ = requireParameter<double>(
        node_, "runtime/gps_message_max_age_sec");
    global_odometry_timeout_sec_ = requireParameter<double>(
        node_, "runtime/global_odometry_timeout_sec");
    max_future_stamp_sec_ = requireParameter<double>(
        node_, "runtime/max_future_stamp_sec");
    timeout_check_period_sec_ = requireParameter<double>(
        node_, "runtime/timeout_check_period_sec");

    fallback_xy_variance_m2_ = requireParameter<double>(
        node_, "gps_covariance/fallback_xy_variance_m2");
    unobserved_variance_ = requireParameter<double>(
        node_, "gps_covariance/unobserved_variance");
    min_quaternion_norm_ = requireParameter<double>(
        node_, "validation/min_quaternion_norm");

    required_fixes_ = requireParameter<int>(
        node_, "reacquisition/required_consecutive_fixes");
    max_innovation_m_ = requireParameter<double>(
        node_, "reacquisition/max_innovation_m");
    stable_return_radius_m_ = requireParameter<double>(
        node_, "reacquisition/stable_return_radius_m");
    reset_xy_variance_m2_ = requireParameter<double>(
        node_, "reacquisition/reset_xy_variance_m2");
    reset_unobserved_variance_ = requireParameter<double>(
        node_, "reacquisition/reset_unobserved_variance");
    set_pose_call_timeout_sec_ = requireParameter<double>(
        node_, "reacquisition/set_pose_call_timeout_sec");

    requireAbsoluteName(gps_fix_topic_, "topics/input_gps_fix");
    requireAbsoluteName(gps_pose_topic_, "topics/gps_map_pose");
    requireAbsoluteName(gps_state_topic_, "topics/gps_state");
    requireAbsoluteName(global_odometry_topic_,
                        "topics/global_filtered_odometry");
    requireAbsoluteName(set_pose_service_, "services/global_set_pose");
    if (map_frame_.empty() || base_link_frame_.empty()) {
      throw std::runtime_error(
          "frames/map and frames/base_link must not be empty");
    }
    if (utm_zone_ < 1 || utm_zone_ > 60) {
      throw std::runtime_error("projection/utm_zone must be between 1 and 60");
    }
    if (!std::isfinite(origin_easting_m_) ||
        !std::isfinite(origin_northing_m_) || !std::isfinite(map_z_)) {
      throw std::runtime_error("projection origin values must be finite");
    }
    requireFinitePositive(gps_timeout_sec_, "runtime/gps_timeout_sec");
    requireFinitePositive(gps_message_max_age_sec_,
                          "runtime/gps_message_max_age_sec");
    requireFinitePositive(global_odometry_timeout_sec_,
                          "runtime/global_odometry_timeout_sec");
    requireFiniteNonnegative(max_future_stamp_sec_,
                             "runtime/max_future_stamp_sec");
    requireFinitePositive(timeout_check_period_sec_,
                          "runtime/timeout_check_period_sec");
    requireFinitePositive(fallback_xy_variance_m2_,
                          "gps_covariance/fallback_xy_variance_m2");
    requireFinitePositive(unobserved_variance_,
                          "gps_covariance/unobserved_variance");
    requireFinitePositive(min_quaternion_norm_,
                          "validation/min_quaternion_norm");
    if (required_fixes_ <= 0) {
      throw std::runtime_error(
          "reacquisition/required_consecutive_fixes must be positive");
    }
    requireFiniteNonnegative(max_innovation_m_,
                             "reacquisition/max_innovation_m");
    requireFiniteNonnegative(stable_return_radius_m_,
                             "reacquisition/stable_return_radius_m");
    requireFinitePositive(reset_xy_variance_m2_,
                          "reacquisition/reset_xy_variance_m2");
    requireFinitePositive(reset_unobserved_variance_,
                          "reacquisition/reset_unobserved_variance");
    requireFinitePositive(set_pose_call_timeout_sec_,
                          "reacquisition/set_pose_call_timeout_sec");
  }

  // 함수이름: validStamp
  // 기능: GPS timestamp가 fresh하고 이전 승인 fix보다 증가했는지 검사한다.
  // 인자: stamp - 검사할 GPS ROS timestamp
  // 반환값: timestamp가 유효하면 true, 아니면 false
  bool validStamp(const ros::Time& stamp) const {
    const ros::Time now = ros::Time::now();
    if (!timestampIsFresh(stamp.toSec(), now.toSec(),
                          gps_message_max_age_sec_, max_future_stamp_sec_)) {
      return false;
    }
    return !have_last_fix_stamp_ || stamp > last_fix_stamp_;
  }

  // 함수이름: validOdometryStamp
  // 기능: Global EKF timestamp가 fresh하고 이전 prediction보다
  //       증가했는지 검사한다.
  // 인자: stamp - 검사할 odometry ROS timestamp
  // 반환값: timestamp가 유효하면 true, 아니면 false
  bool validOdometryStamp(const ros::Time& stamp) const {
    const ros::Time now = ros::Time::now();
    if (!timestampIsFresh(stamp.toSec(), now.toSec(),
                          global_odometry_timeout_sec_,
                          max_future_stamp_sec_)) {
      return false;
    }
    return !have_last_odometry_stamp_ || stamp > last_odometry_stamp_;
  }

  // 함수이름: covarianceFromFix
  // 기능: NavSatFix covariance에서 유효한 X/Y 분산을 읽거나 fallback을 적용한다.
  // 인자: fix, variance_x, variance_y - 결과를 받을 포인터
  // 반환값: 사용할 분산을 얻었으면 true, covariance가 잘못됐으면 false
  bool covarianceFromFix(const sensor_msgs::NavSatFix& fix,
                         double* variance_x, double* variance_y) const {
    if (variance_x == nullptr || variance_y == nullptr) {
      return false;
    }
    if (fix.position_covariance_type ==
        sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      *variance_x = fallback_xy_variance_m2_;
      *variance_y = fallback_xy_variance_m2_;
      ROS_WARN_THROTTLE(5.0,
                        "GPS covariance is unknown; using configured fallback");
      return true;
    }
    if (fix.position_covariance_type >
        sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN) {
      return false;
    }
    const double x = fix.position_covariance[0];
    const double y = fix.position_covariance[4];
    if (!std::isfinite(x) || !std::isfinite(y) || x < 0.0 || y < 0.0) {
      return false;
    }
    *variance_x = x;
    *variance_y = y;
    return true;
  }

  // 함수이름: makePose
  // 기능: 투영된 map 위치와 GPS 분산을 Global EKF 입력 Pose 메시지로 만든다.
  // 인자: fix, point, variance_x, variance_y
  // 반환값: map frame의 PoseWithCovarianceStamped
  geometry_msgs::PoseWithCovarianceStamped makePose(
      const sensor_msgs::NavSatFix& fix, const MapPoint& point,
      double variance_x, double variance_y) const {
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.stamp = fix.header.stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = point.x;
    pose.pose.pose.position.y = point.y;
    pose.pose.pose.position.z = point.z;
    pose.pose.pose.orientation.w = 1.0;
    std::fill(pose.pose.covariance.begin(), pose.pose.covariance.end(), 0.0);
    pose.pose.covariance[0] = variance_x;
    pose.pose.covariance[7] = variance_y;
    pose.pose.covariance[14] = unobserved_variance_;
    pose.pose.covariance[21] = unobserved_variance_;
    pose.pose.covariance[28] = unobserved_variance_;
    pose.pose.covariance[35] = unobserved_variance_;
    return pose;
  }

  // 함수이름: predictionIsFresh
  // 기능: 마지막 Global EKF prediction의 timestamp와 수신 시간이
  //       fresh한지 확인한다.
  // 인자: 없음
  // 반환값: prediction을 GPS gate와 SetPose에 사용할 수 있으면 true, 아니면 false
  bool predictionIsFresh() const {
    if (!have_prediction_ || !have_last_odometry_stamp_) {
      return false;
    }
    const ros::Time now = ros::Time::now();
    return timestampIsFresh(last_odometry_stamp_.toSec(), now.toSec(),
                            global_odometry_timeout_sec_,
                            max_future_stamp_sec_) &&
           receiptIsFresh(last_prediction_receipt_.toSec(),
                          ros::SteadyTime::now().toSec(),
                          global_odometry_timeout_sec_);
  }

  // 함수이름: beginGlobalFilterReset
  // 기능: 안정화된 GPS pose로 Global EKF SetPose를 비동기 호출한다.
  // 인자: pose - reset 기준으로 사용할 GPS map pose
  // 반환값: 비동기 호출을 시작했으면 true, 시작 조건을 만족하지 않으면
  //         false
  bool beginGlobalFilterReset(
      const geometry_msgs::PoseWithCovarianceStamped& gps_pose) {
    if (reset_call_inflight_) {
      return true;
    }
    if (!predictionIsFresh()) {
      ROS_WARN_THROTTLE(1.0,
                        "Cannot reset global filter without fresh odometry");
      return false;
    }
    robot_localization::SetPose service;
    service.request.pose = gps_pose;
    service.request.pose.pose.pose.orientation = prediction_orientation_;
    service.request.pose.pose.covariance[0] = std::max(
        gps_pose.pose.covariance[0], reset_xy_variance_m2_);
    service.request.pose.pose.covariance[7] = std::max(
        gps_pose.pose.covariance[7], reset_xy_variance_m2_);
    service.request.pose.pose.covariance[14] = reset_unobserved_variance_;
    service.request.pose.pose.covariance[21] = reset_unobserved_variance_;
    service.request.pose.pose.covariance[28] = reset_unobserved_variance_;
    service.request.pose.pose.covariance[35] = reset_unobserved_variance_;
    reset_call_result_.reset(new ResetCallResult());
    pending_reset_pose_ = gps_pose;
    pending_reset_gps_receipt_ = last_candidate_fix_receipt_;
    reset_call_started_ = ros::SteadyTime::now();
    reset_call_inflight_ = true;
    reset_call_timeout_reported_ = false;

    const std::shared_ptr<ResetCallResult> result = reset_call_result_;
    const ros::SteadyTime call_started = reset_call_started_;
    const ros::Time gps_stamp = gps_pose.header.stamp;
    const ros::Time odometry_stamp = last_odometry_stamp_;
    const ros::SteadyTime gps_receipt = pending_reset_gps_receipt_;
    const ros::SteadyTime prediction_receipt = last_prediction_receipt_;
    const double gps_message_max_age_sec = gps_message_max_age_sec_;
    const double global_odometry_timeout_sec = global_odometry_timeout_sec_;
    const double max_future_stamp_sec = max_future_stamp_sec_;
    ros::ServiceClient client = set_pose_client_;
    std::thread([client, service, result, call_started,
                 gps_stamp, odometry_stamp, gps_receipt, prediction_receipt,
                 gps_message_max_age_sec, global_odometry_timeout_sec,
                 max_future_stamp_sec]() mutable {
      bool success = false;
      if (client.exists() && resetSnapshotIsFresh(
                                 gps_stamp.toSec(), odometry_stamp.toSec(),
                                 gps_receipt.toSec(),
                                 prediction_receipt.toSec(),
                                 ros::Time::now().toSec(),
                                 ros::SteadyTime::now().toSec(),
                                 gps_message_max_age_sec,
                                 global_odometry_timeout_sec,
                                 max_future_stamp_sec)) {
        success = client.call(service);
      }
      result->success.store(success);
      result->completion_elapsed_sec.store(
          (ros::SteadyTime::now() - call_started).toSec());
      result->done.store(true);
    }).detach();
    return true;
  }

  // 함수이름: finishResetCall
  // 기능: 완료된 SetPose 공유 결과와 진행 상태를 초기화한다.
  // 인자: 없음
  // 반환값: 없음
  void finishResetCall() {
    reset_call_result_.reset();
    reset_call_inflight_ = false;
    reset_call_timeout_reported_ = false;
  }

  // 함수이름: pollResetCall
  // 기능: 비동기 SetPose 결과와 snapshot freshness를 확인해 gate 상태를
  //       반영한다.
  // 인자: 없음
  // 반환값: 없음
  void pollResetCall() {
    if (!reset_call_inflight_ || !reset_call_result_) {
      return;
    }
    const double elapsed =
        (ros::SteadyTime::now() - reset_call_started_).toSec();
    if (!reset_call_result_->done.load()) {
      if (!reset_call_timeout_reported_ &&
          elapsed > set_pose_call_timeout_sec_) {
        reset_call_timeout_reported_ = true;
        publishState(true);
        ROS_ERROR("SetPose response is overdue; remaining quarantined in "
                  "RELOCALIZING until the RPC resolves");
      }
      return;
    }

    const double completion_elapsed_sec =
        reset_call_result_->completion_elapsed_sec.load();
    if (completion_elapsed_sec > set_pose_call_timeout_sec_) {
      ROS_WARN("Reconciling SetPose response received after %.3f seconds",
               completion_elapsed_sec);
    }

    const bool succeeded = reset_call_result_->success.load();
    const bool snapshot_is_still_fresh = resetSnapshotIsFresh(
        pending_reset_pose_.header.stamp.toSec(), last_odometry_stamp_.toSec(),
        pending_reset_gps_receipt_.toSec(), last_prediction_receipt_.toSec(),
        ros::Time::now().toSec(),
        ros::SteadyTime::now().toSec(), gps_message_max_age_sec_,
        global_odometry_timeout_sec_, max_future_stamp_sec_);
    if (succeeded && !snapshot_is_still_fresh) {
      gate_->markTimeout();
      publishState(false);
      ROS_ERROR("Discarding stale SetPose completion; fresh GPS fixes are "
                "required before tracking resumes");
      finishResetCall();
      return;
    }
    gate_->confirmReset(succeeded);
    publishState(false);
    if (succeeded && gate_->stateName() == "TRACKING") {
      pose_publisher_.publish(pending_reset_pose_);
      last_accepted_fix_receipt_ = ros::SteadyTime::now();
      have_accepted_fix_ = true;
    } else if (!succeeded) {
      ROS_ERROR("Configured SetPose service call failed");
    }
    finishResetCall();
  }

  // 함수이름: fixCallback
  // 기능: GPS fix를 검증·투영하고 gate 결정에 따라 pose 발행 또는
  //       SetPose를 시작한다.
  // 인자: fix - 수신한 sensor_msgs/NavSatFix
  // 반환값: 없음
  void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& fix) {
    if (fix->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
      ROS_WARN_THROTTLE(1.0, "Rejecting GPS message without a valid fix");
      return;
    }
    if (!validStamp(fix->header.stamp)) {
      ROS_WARN_THROTTLE(1.0, "Rejecting GPS message with invalid timestamp");
      return;
    }

    double variance_x = 0.0;
    double variance_y = 0.0;
    if (!covarianceFromFix(*fix, &variance_x, &variance_y)) {
      ROS_WARN_THROTTLE(1.0, "Rejecting GPS message with invalid covariance");
      return;
    }

    MapPoint point;
    std::string projection_error;
    if (!projector_->project(fix->latitude, fix->longitude, fix->altitude,
                             &point, &projection_error)) {
      ROS_WARN_THROTTLE(1.0, "Rejecting GPS coordinates: %s",
                        projection_error.c_str());
      return;
    }

    last_fix_stamp_ = fix->header.stamp;
    have_last_fix_stamp_ = true;
    last_candidate_fix_receipt_ = ros::SteadyTime::now();
    have_candidate_fix_ = true;

    // SetPose 응답을 기다리는 동안 gate의 reset_pending 계약을 고정한다.
    // 후보 fix의 수신 시간과 timestamp는 계속 갱신하여 service 완료 뒤 재사용을 막는다.
    if (reset_call_inflight_) {
      return;
    }

    const bool have_prediction = predictionIsFresh();
    const GateDecision decision = gate_->evaluate(
        point, have_prediction, prediction_x_, prediction_y_);
    publishState(false);

    const geometry_msgs::PoseWithCovarianceStamped pose = makePose(
        *fix, point, variance_x, variance_y);
    if (decision.action == GpsGateAction::ACCEPT) {
      pose_publisher_.publish(pose);
      last_accepted_fix_receipt_ = ros::SteadyTime::now();
      have_accepted_fix_ = true;
      return;
    }
    if (decision.action == GpsGateAction::RESET_FILTER) {
      if (!beginGlobalFilterReset(pose)) {
        gate_->confirmReset(false);
        publishState(false);
      }
    }
  }

  // 함수이름: odometryCallback
  // 기능: Global EKF odometry를 검증해 GPS gate와 SetPose용 최신
  //       prediction으로 저장한다.
  // 인자: odometry - 수신한 nav_msgs/Odometry
  // 반환값: 없음
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry) {
    geometry_msgs::Quaternion normalized_orientation;
    if (!validOdometryStamp(odometry->header.stamp) ||
        odometry->header.frame_id != map_frame_ ||
        odometry->child_frame_id != base_link_frame_ ||
        !std::isfinite(odometry->pose.pose.position.x) ||
        !std::isfinite(odometry->pose.pose.position.y) ||
        !normalizeQuaternion(odometry->pose.pose.orientation,
                             min_quaternion_norm_,
                             &normalized_orientation)) {
      ROS_WARN_THROTTLE(1.0,
                        "Rejecting invalid global-filter odometry prediction");
      return;
    }
    prediction_x_ = odometry->pose.pose.position.x;
    prediction_y_ = odometry->pose.pose.position.y;
    prediction_orientation_ = normalized_orientation;
    last_prediction_receipt_ = ros::SteadyTime::now();
    last_odometry_stamp_ = odometry->header.stamp;
    have_last_odometry_stamp_ = true;
    have_prediction_ = true;
  }

  // 함수이름: timeoutCallback
  // 기능: SetPose 결과와 GPS 수신 timeout을 주기적으로 확인하고 gate
  //       상태를 발행한다.
  // 인자: ros::SteadyTimerEvent - 사용하지 않음
  // 반환값: 없음
  void timeoutCallback(const ros::SteadyTimerEvent&) {
    // Supervisor가 projector의 생존 여부를 확인할 수 있도록 현재 상태를 주기적으로 발행한다.
    publishState(true);
    pollResetCall();
    if (reset_call_inflight_) {
      return;
    }
    const std::string state = gate_->stateName();
    ros::SteadyTime reference;
    if (state == "TRACKING" && have_accepted_fix_) {
      reference = last_accepted_fix_receipt_;
    } else if (state == "RELOCALIZING" && have_candidate_fix_) {
      reference = last_candidate_fix_receipt_;
    } else {
      return;
    }
    if ((ros::SteadyTime::now() - reference).toSec() <= gps_timeout_sec_) {
      return;
    }
    gate_->markTimeout();
    publishState(false);
  }

  // 함수이름: publishState
  // 기능: gate 상태가 바뀌었거나 강제 발행이 요청되면 latched 상태
  //       topic을 발행한다.
  // 인자: force - 상태가 같아도 발행할지 여부
  // 반환값: 없음
  void publishState(bool force) {
    const std::string state = gate_->stateName();
    if (!force && state == last_published_state_) {
      return;
    }
    std_msgs::String message;
    message.data = state;
    state_publisher_.publish(message);
    last_published_state_ = state;
  }

  ros::NodeHandle node_;
  ros::Publisher pose_publisher_;
  ros::Publisher state_publisher_;
  ros::Subscriber fix_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::ServiceClient set_pose_client_;
  ros::SteadyTimer timeout_timer_;

  std::unique_ptr<GpsProjection> projector_;
  std::unique_ptr<GpsGate> gate_;

  std::string gps_fix_topic_;
  std::string gps_pose_topic_;
  std::string gps_state_topic_;
  std::string global_odometry_topic_;
  std::string set_pose_service_;
  std::string map_frame_;
  std::string base_link_frame_;
  std::string last_published_state_;

  int utm_zone_;
  bool north_hemisphere_;
  double origin_easting_m_;
  double origin_northing_m_;
  double map_z_;
  double gps_timeout_sec_;
  double gps_message_max_age_sec_;
  double global_odometry_timeout_sec_;
  double max_future_stamp_sec_;
  double timeout_check_period_sec_;
  double fallback_xy_variance_m2_;
  double unobserved_variance_;
  double min_quaternion_norm_;
  int required_fixes_;
  double max_innovation_m_;
  double stable_return_radius_m_;
  double reset_xy_variance_m2_;
  double reset_unobserved_variance_;
  double set_pose_call_timeout_sec_;

  bool have_prediction_;
  double prediction_x_;
  double prediction_y_;
  geometry_msgs::Quaternion prediction_orientation_;
  ros::SteadyTime last_prediction_receipt_;

  bool have_accepted_fix_;
  ros::SteadyTime last_accepted_fix_receipt_;
  bool have_candidate_fix_;
  ros::SteadyTime last_candidate_fix_receipt_;
  bool have_last_fix_stamp_;
  ros::Time last_fix_stamp_;
  bool have_last_odometry_stamp_;
  ros::Time last_odometry_stamp_;

  std::shared_ptr<ResetCallResult> reset_call_result_;
  geometry_msgs::PoseWithCovarianceStamped pending_reset_pose_;
  ros::SteadyTime pending_reset_gps_receipt_;
  ros::SteadyTime reset_call_started_;
  bool reset_call_inflight_;
  bool reset_call_timeout_reported_;
};

}  // namespace localization_pkg

// 함수이름: main
// 기능: GPS projector ROS node를 초기화하고 callback 처리를 시작한다.
// 인자: argc, argv - ROS command-line 인자
// 반환값: 정상 종료는 0, 초기화 실패는 1
int main(int argc, char** argv) {
  ros::init(argc, argv, "molit_gps_projector");
  try {
    localization_pkg::GpsProjectorNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("GPS projector startup failed: %s", exception.what());
    return 1;
  }
  return 0;
}

