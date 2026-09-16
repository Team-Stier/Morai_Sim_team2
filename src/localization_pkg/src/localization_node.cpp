/*
 * localization_node.cpp
 *
 * Diagnostic-only runtime node for localization_pkg.
 *
 * This node intentionally publishes only /molit/localization/status while
 * TF/map/timestamp readiness gates remain blocked.
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <string>

#include <common_msgs_pkg/LocalizationStatus.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <yaml-cpp/yaml.h>

namespace {

constexpr char kContractPackage[] = "ros_architecture_pkg";
constexpr char kFrameContractPath[] = "config/tf/frame_contract.yaml";
constexpr char kTimestampContractPath[] = "config/timestamp/timestamp_contract.yaml";

constexpr char kExpectedGpsFrame[] = "gps_link";
constexpr char kExpectedImuFrame[] = "imu_link";

constexpr double kExpectedStatusPublishRateHz = 10.0;
constexpr double kDefaultClockStallSec = 0.5;
constexpr double kStatusRateTolerance = 1e-6;
constexpr std::size_t kNavSatCovSize = 9;

std::string formatTimeSec(const ros::Time& stamp) {
  if (stamp.isZero()) {
    return "0";
  }
  std::ostringstream ss;
  ss << stamp.toSec();
  return ss.str();
}

bool isFinite(double value) {
  return std::isfinite(value);
}

enum class RejectReasonType {
  kDuplicate,
  kRegression,
  kOther,
};

RejectReasonType classifyRejectReason(const std::string& reason) {
  if (reason.find("duplicate") != std::string::npos) {
    return RejectReasonType::kDuplicate;
  }
  if (reason.find("regression") != std::string::npos) {
    return RejectReasonType::kRegression;
  }
  return RejectReasonType::kOther;
}

struct SourceState {
  bool has_accepted_stamp = false;
  ros::Time last_accepted_stamp = ros::Time(0);
  std::uint32_t accepted_count = 0;
  std::uint32_t rejected_count = 0;
  std::uint32_t duplicate_count = 0;
  std::uint32_t regression_count = 0;

  void reset() {
    has_accepted_stamp = false;
    last_accepted_stamp = ros::Time(0);
    accepted_count = 0;
    rejected_count = 0;
    duplicate_count = 0;
    regression_count = 0;
  }

  void accept(const ros::Time& sample_stamp) {
    has_accepted_stamp = true;
    last_accepted_stamp = sample_stamp;
    ++accepted_count;
  }

  void reject(bool duplicate, bool regression) {
    ++rejected_count;
    if (duplicate) {
      ++duplicate_count;
    }
    if (regression) {
      ++regression_count;
    }
  }
};

struct ClockState {
  bool has_reference = false;
  ros::Time last_ros_time = ros::Time(0);
  std::chrono::steady_clock::time_point last_wall_time;
  bool clock_running = false;
  bool stall_event = false;
  bool regression_reason_pending = false;
  std::uint32_t reset_id = 0;
  std::string regression_reason;
};

}  // namespace

class LocalizationDiagnosticNode {
 public:
  LocalizationDiagnosticNode() : node_handle_(), private_node_handle_("~") {}

  bool initialize() {
    if (!loadCentralContracts()) {
      return false;
    }

    if (!validateAndBindParams()) {
      return false;
    }

    status_publisher_ = node_handle_.advertise<common_msgs_pkg::LocalizationStatus>(
        "/molit/localization/status", 1, true);
    gps_subscriber_ = node_handle_.subscribe("/molit/sensors/gps/fix", 10,
                                             &LocalizationDiagnosticNode::onGps, this);
    imu_subscriber_ = node_handle_.subscribe("/molit/sensors/imu/data", 10,
                                             &LocalizationDiagnosticNode::onImu, this);

    status_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(1.0 / status_publish_rate_hz_),
        &LocalizationDiagnosticNode::publishStatus, this);
    return true;
  }

 private:
  bool loadCentralContracts() {
    const std::string package_path = ros::package::getPath(kContractPackage);
    if (package_path.empty()) {
      ROS_ERROR("Cannot resolve path for %s", kContractPackage);
      return false;
    }
    const std::string frame_path = package_path + std::string("/") + kFrameContractPath;
    const std::string timestamp_path = package_path + std::string("/") + kTimestampContractPath;

    try {
      const YAML::Node frame_contract = YAML::LoadFile(frame_path);
      const YAML::Node timestamp_contract = YAML::LoadFile(timestamp_path);
      if (!validateFrameContract(frame_contract)) {
        return false;
      }
      if (!validateTimestampContract(timestamp_contract)) {
        return false;
      }
    } catch (const std::exception& err) {
      ROS_ERROR("Failed loading central contracts: %s", err.what());
      return false;
    }
    ROS_INFO("Central contracts loaded successfully. Node stays in diagnostic mode.");
    return true;
  }

  bool validateFrameContract(const YAML::Node& frame_contract) {
    if (!frame_contract["transforms"] || !frame_contract["transforms"].IsSequence()) {
      ROS_ERROR("frame_contract malformed: transforms must be a sequence.");
      return false;
    }

    bool map_to_odom_blocked = false;
    bool odom_to_base_blocked = false;
    for (const YAML::Node& transform : frame_contract["transforms"]) {
      if (!transform["parent"] || !transform["child"] ||
          !transform["publish_enabled"].IsDefined()) {
        continue;
      }
      const std::string parent = transform["parent"].as<std::string>("");
      const std::string child = transform["child"].as<std::string>("");
      if (parent == "map" && child == "odom") {
        map_to_odom_blocked = true;  // Diagnostic mode never publishes TF.
      } else if (parent == "odom" && child == "base_link") {
        odom_to_base_blocked = true;
      }
    }

    if (!map_to_odom_blocked || !odom_to_base_blocked) {
      ROS_ERROR("Central frame gate remains active: map->odom=%s odom->base_link=%s",
                map_to_odom_blocked ? "blocked" : "unverified",
                odom_to_base_blocked ? "blocked" : "unverified");
      return false;
    }
    return true;
  }

  bool validateTimestampContract(const YAML::Node& timestamp_contract) {
    if (!timestamp_contract["header_stamp_contract"] || !timestamp_contract["header_stamp_contract"].IsMap()) {
      ROS_ERROR("timestamp_contract malformed: header_stamp_contract missing or not a map.");
      return false;
    }
    if (!timestamp_contract["timestamp_source_registry"] ||
        !timestamp_contract["timestamp_source_registry"].IsMap()) {
      ROS_ERROR("timestamp_contract malformed: timestamp_source_registry missing or not a map.");
      return false;
    }
    const YAML::Node source_registry = timestamp_contract["timestamp_source_registry"];
    if (!source_registry["status_evaluation_time"] || !source_registry["status_evaluation_time"].IsMap()) {
      ROS_ERROR("timestamp_contract malformed: timestamp_source_registry/status_evaluation_time missing.");
      return false;
    }
    const YAML::Node status_eval = source_registry["status_evaluation_time"];
    if (!status_eval["clock_domain"] || status_eval["clock_domain"].as<std::string>("") != "canonical_message_clock") {
      ROS_ERROR("timestamp_contract malformed: status_evaluation_time.clock_domain must be canonical_message_clock.");
      return false;
    }
    if (!status_eval["allowed_topics"] || !status_eval["allowed_topics"].IsSequence()) {
      ROS_ERROR("timestamp_contract malformed: status_evaluation_time.allowed_topics must be sequence.");
      return false;
    }
    bool status_topic_allowed = false;
    for (const auto& topic : status_eval["allowed_topics"]) {
      if (topic.as<std::string>("") == "/molit/localization/status") {
        status_topic_allowed = true;
        break;
      }
    }
    if (!status_topic_allowed) {
      ROS_ERROR("timestamp_contract malformed: /molit/localization/status not in status_evaluation_time allowed_topics.");
      return false;
    }

    if (!timestamp_contract["ordering_and_reset_policy"] ||
        !timestamp_contract["ordering_and_reset_policy"].IsMap()) {
      ROS_ERROR("timestamp_contract malformed: ordering_and_reset_policy missing or not a map.");
      return false;
    }
    const YAML::Node reset_policy = timestamp_contract["ordering_and_reset_policy"];
    if (!reset_policy["clock_reset"] || reset_policy["clock_reset"].as<std::string>("") != "clear_temporal_buffers_and_reinitialize") {
      ROS_ERROR("timestamp_contract malformed: ordering_and_reset_policy.clock_reset mismatch.");
      return false;
    }
    if (!reset_policy["per_source_regression"] || reset_policy["per_source_regression"].as<std::string>("") != "reject_and_report") {
      ROS_ERROR("timestamp_contract malformed: ordering_and_reset_policy.per_source_regression mismatch.");
      return false;
    }
    if (!reset_policy["cross_clock_comparison"] ||
        reset_policy["cross_clock_comparison"].as<std::string>("") != "forbidden") {
      ROS_ERROR("timestamp_contract malformed: ordering_and_reset_policy.cross_clock_comparison must be forbidden.");
      return false;
    }

    return true;
  }

  bool validateAndBindParams() {
    private_node_handle_.param("status_publish_rate_hz", status_publish_rate_hz_, kExpectedStatusPublishRateHz);
    private_node_handle_.param("clock_stall_sec", clock_stall_sec_, kDefaultClockStallSec);

    if (!isFinite(status_publish_rate_hz_) ||
        std::abs(status_publish_rate_hz_ - kExpectedStatusPublishRateHz) > kStatusRateTolerance) {
      ROS_ERROR("status_publish_rate_hz must be a finite value equal to %.3f", kExpectedStatusPublishRateHz);
      return false;
    }

    if (!isFinite(clock_stall_sec_) || clock_stall_sec_ <= 0.0) {
      ROS_ERROR("clock_stall_sec must be finite and > 0");
      return false;
    }

    return true;
  }

  void updateClockState(const ros::Time& now_ros) {
    const auto now_wall = std::chrono::steady_clock::now();
    clock_state_.stall_event = false;

    if (!clock_state_.has_reference) {
      clock_state_.has_reference = true;
      clock_state_.last_ros_time = now_ros;
      clock_state_.last_wall_time = now_wall;
      clock_state_.clock_running = !now_ros.isZero();
      return;
    }

    if (now_ros < clock_state_.last_ros_time) {
      ++clock_state_.reset_id;
      clock_state_.regression_reason_pending = true;
      clock_state_.clock_running = false;
      clock_state_.regression_reason = "ROS clock regressed (time decreased); temporal buffers reset.";
      clock_state_.last_ros_time = now_ros;
      clock_state_.last_wall_time = now_wall;
      gps_state_.reset();
      imu_state_.reset();
      last_gps_reject_reason_.clear();
      last_imu_reject_reason_.clear();
      ROS_WARN("Localization diagnostic: ROS clock regression detected, reset_id=%u", clock_state_.reset_id);
      return;
    }

    if (now_ros == clock_state_.last_ros_time) {
      const double wall_delta_sec =
          std::chrono::duration_cast<std::chrono::duration<double>>(now_wall - clock_state_.last_wall_time)
              .count();
      if (wall_delta_sec >= clock_stall_sec_) {
        clock_state_.clock_running = false;
        clock_state_.stall_event = true;
      }
      return;
    }

    clock_state_.clock_running = true;
    clock_state_.last_ros_time = now_ros;
    clock_state_.last_wall_time = now_wall;
  }

  bool validateGpsMessage(const sensor_msgs::NavSatFix& msg, const ros::Time& now_ros,
                         std::string* reason) {
    if (msg.header.frame_id.empty()) {
      *reason = "gps frame is empty";
      return false;
    }
    if (msg.header.frame_id != kExpectedGpsFrame) {
      *reason = "gps frame mismatch (expected gps_link)";
      return false;
    }
    if (msg.header.stamp.isZero()) {
      *reason = "gps header stamp is zero";
      return false;
    }
    if (msg.header.stamp > now_ros) {
      *reason = "gps stamp in the future";
      return false;
    }

    if (msg.status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX || msg.status.status < 0) {
      *reason = "gps status indicates no-fix";
      return false;
    }

    const double lat = msg.latitude;
    const double lon = msg.longitude;
    const double alt = msg.altitude;
    if (!isFinite(lat) || !isFinite(lon) || !isFinite(alt)) {
      *reason = "gps has non-finite fields";
      return false;
    }
    if (lat < -90.0 || lat > 90.0) {
      *reason = "gps latitude out of range";
      return false;
    }
    if (lon < -180.0 || lon > 180.0) {
      *reason = "gps longitude out of range";
      return false;
    }

    if (msg.position_covariance.size() != kNavSatCovSize) {
      *reason = "gps covariance must be 9-element matrix";
      return false;
    }
    for (std::size_t i = 0; i < msg.position_covariance.size(); ++i) {
      if (!isFinite(msg.position_covariance[i])) {
        *reason = "gps covariance contains non-finite value";
        return false;
      }
    }
    if (msg.position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      const double x_var = msg.position_covariance[0];
      const double y_var = msg.position_covariance[4];
      const double z_var = msg.position_covariance[8];
      if (x_var <= 0.0 || y_var <= 0.0 || z_var <= 0.0) {
        *reason = "gps covariance diagonal must be positive";
        return false;
      }
    }

    if (gps_state_.has_accepted_stamp) {
      if (msg.header.stamp == gps_state_.last_accepted_stamp) {
        *reason = "gps duplicate stamp";
        return false;
      }
      if (msg.header.stamp < gps_state_.last_accepted_stamp) {
        *reason = "gps stamp regression";
        return false;
      }
    }
    return true;
  }

  bool validateImuMessage(const sensor_msgs::Imu& msg, const ros::Time& now_ros,
                         std::string* reason) {
    if (msg.header.frame_id.empty()) {
      *reason = "imu frame is empty";
      return false;
    }
    if (msg.header.frame_id != kExpectedImuFrame) {
      *reason = "imu frame mismatch (expected imu_link)";
      return false;
    }
    if (msg.header.stamp.isZero()) {
      *reason = "imu header stamp is zero";
      return false;
    }
    if (msg.header.stamp > now_ros) {
      *reason = "imu stamp in the future";
      return false;
    }

    const double ax = msg.linear_acceleration.x;
    const double ay = msg.linear_acceleration.y;
    const double az = msg.linear_acceleration.z;
    const double wx = msg.angular_velocity.x;
    const double wy = msg.angular_velocity.y;
    const double wz = msg.angular_velocity.z;
    if (!isFinite(ax) || !isFinite(ay) || !isFinite(az) || !isFinite(wx) || !isFinite(wy) ||
        !isFinite(wz)) {
      *reason = "imu contains non-finite data";
      return false;
    }

    if (imu_state_.has_accepted_stamp) {
      if (msg.header.stamp == imu_state_.last_accepted_stamp) {
        *reason = "imu duplicate stamp";
        return false;
      }
      if (msg.header.stamp < imu_state_.last_accepted_stamp) {
        *reason = "imu stamp regression";
        return false;
      }
    }
    return true;
  }

  void onGps(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const ros::Time now_ros = ros::Time::now();
    updateClockState(now_ros);

    if (now_ros.isZero()) {
      return;
    }

    std::string reason;
    if (!validateGpsMessage(*msg, now_ros, &reason)) {
      const auto type = classifyRejectReason(reason);
      const bool duplicate = type == RejectReasonType::kDuplicate;
      const bool regression = type == RejectReasonType::kRegression;
      gps_state_.reject(duplicate, regression);
      last_gps_reject_reason_ = reason;
      return;
    }

    gps_state_.accept(msg->header.stamp);
    last_gps_reject_reason_.clear();
  }

  void onImu(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const ros::Time now_ros = ros::Time::now();
    updateClockState(now_ros);

    if (now_ros.isZero()) {
      return;
    }

    std::string reason;
    if (!validateImuMessage(*msg, now_ros, &reason)) {
      const auto type = classifyRejectReason(reason);
      const bool duplicate = type == RejectReasonType::kDuplicate;
      const bool regression = type == RejectReasonType::kRegression;
      imu_state_.reject(duplicate, regression);
      last_imu_reject_reason_ = reason;
      return;
    }

    imu_state_.accept(msg->header.stamp);
    last_imu_reject_reason_.clear();
  }

  void publishStatus(const ros::WallTimerEvent&) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const ros::Time now_ros = ros::Time::now();
    updateClockState(now_ros);

    if (now_ros.isZero()) {
      return;
    }

    common_msgs_pkg::LocalizationStatus status;
    status.header.stamp = now_ros;
    status.header.frame_id.clear();
    status.map_pose_valid = false;
    status.local_odometry_valid = false;
    status.stop_required = true;
    status.ego_state_stamp = ros::Time(0);
    status.local_odometry_stamp = ros::Time(0);
    status.reset_id = clock_state_.reset_id;
    status.map_position_stddev_m = -1.0;
    status.local_position_stddev_m = -1.0;
    status.yaw_stddev_rad = -1.0;

    if (!clock_state_.clock_running) {
      status.mode = common_msgs_pkg::LocalizationStatus::LOST;
    } else if (!gps_state_.has_accepted_stamp && !imu_state_.has_accepted_stamp) {
      status.mode = common_msgs_pkg::LocalizationStatus::UNINITIALIZED;
    } else {
      status.mode = common_msgs_pkg::LocalizationStatus::INITIALIZING;
    }

    status.gps_fix_valid = false;
    status.gps_age_sec = -1.0;
    if (gps_state_.has_accepted_stamp) {
      const double age = now_ros.toSec() - gps_state_.last_accepted_stamp.toSec();
      if (age >= 0.0) {
        status.gps_age_sec = age;
      }
    }

    std::ostringstream reason;
    reason << "Diagnostic-only runtime. map->odom and odom->base_link TF publish gates remain "
           << "locked, so no odometry/EgoState estimates are published.";

    if (!clock_state_.clock_running) {
      if (clock_state_.regression_reason_pending) {
        reason << " Clock regression detected; status LOST.";
      } else {
        reason << " Clock stalled (frozen positive ROS time).";
      }
    }
    if (clock_state_.stall_event) {
      reason << " Clock stall event detected without ROS time advance.";
    }
    if (clock_state_.regression_reason_pending) {
      reason << " " << clock_state_.regression_reason;
    }
    reason << " GPS availability remains unverified until timing policy is approved; "
              "gps_fix_valid stays false. GPS age describes the last accepted diagnostic sample.";
    if (!last_gps_reject_reason_.empty()) {
      reason << " Last GPS reject: " << last_gps_reject_reason_ << ".";
    }
    if (!last_imu_reject_reason_.empty()) {
      reason << " Last IMU reject: " << last_imu_reject_reason_ << ".";
    }
    reason << " gps_accepted=" << gps_state_.accepted_count
           << " gps_rejected=" << gps_state_.rejected_count
           << " gps_duplicates=" << gps_state_.duplicate_count
           << " gps_regressions=" << gps_state_.regression_count
           << " imu_accepted=" << imu_state_.accepted_count
           << " imu_rejected=" << imu_state_.rejected_count
           << " imu_duplicates=" << imu_state_.duplicate_count
           << " imu_regressions=" << imu_state_.regression_count
           << " last_gps_stamp=" << formatTimeSec(gps_state_.last_accepted_stamp)
           << " last_imu_stamp=" << formatTimeSec(imu_state_.last_accepted_stamp);
    status.reason = reason.str();

    status_publisher_.publish(status);
    clock_state_.regression_reason_pending = false;
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Subscriber gps_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Publisher status_publisher_;
  ros::WallTimer status_timer_;

  ClockState clock_state_;
  SourceState gps_state_;
  SourceState imu_state_;
  std::mutex state_mutex_;
  std::string last_gps_reject_reason_;
  std::string last_imu_reject_reason_;

  double status_publish_rate_hz_ = kExpectedStatusPublishRateHz;
  double clock_stall_sec_ = kDefaultClockStallSec;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_node");
  LocalizationDiagnosticNode node;
  if (!node.initialize()) {
    return 1;
  }
  ros::spin();
  return 0;
}
