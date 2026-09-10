#include "detector.h"
#include <common_msgs_pkg/LidarObservationArray.h>
#include <common_msgs_pkg/LidarObjectObservation.h>
#include <common_msgs_pkg/ComponentStatus.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace {
using Steady = std::chrono::steady_clock;
double elapsed(Steady::time_point t) {
  return std::chrono::duration<double>(Steady::now()-t).count();
}
class Node {
  ros::NodeHandle nh_, private_{"~"};
  ros::Subscriber points_, transport_;
  ros::Publisher observations_, status_, debug_;
  ros::WallTimer timer_;
  lidar_perception::Config config_;
  ros::Time last_stamp_, last_clock_;
  Steady::time_point last_received_=Steady::now(), transport_received_=Steady::now();
  bool transport_ok_=false;
  std::string calibration_id_;
  double watchdog_sec_=0, max_scan_age_sec_=0, status_period_sec_=0;
  common_msgs_pkg::ComponentStatus health_;

  void setStatus(const std::string& reason, bool fault=false) {
    const auto now=ros::Time::now();
    health_.header.stamp=now;
    health_.component="lidar_perception_pkg";
    health_.state=fault ? health_.FAULT : health_.DEGRADED;
    health_.ready=false;
    health_.stop_required=true;
    health_.data_stamp=last_stamp_;
    health_.data_age_sec=last_stamp_.isZero() ? -1 : (now-last_stamp_).toSec();
    if (health_.data_age_sec<0) {health_.data_stamp=ros::Time(); health_.data_age_sec=-1;}
    health_.reason=reason;
    // Publishing is owned by the central 2 Hz heartbeat, not scan callbacks.
  }
  void tick(const ros::WallTimerEvent&) {
    const auto now=ros::Time::now();
    if (!last_clock_.isZero() && now<last_clock_) {
      last_stamp_=ros::Time(); transport_ok_=false;
      setStatus("clock reset; waiting for new transport status and scan", true);
    } else if (elapsed(last_received_)>watchdog_sec_ ||
               elapsed(transport_received_)>watchdog_sec_ || !transport_ok_) {
      setStatus("no recent LiDAR transport/scan; old observations are invalid", true);
    } else setStatus(health_.reason, health_.state==health_.FAULT);
    last_clock_=now;
    if (!now.isZero()) status_.publish(health_);
  }
  void transport(const std_msgs::Bool::ConstPtr& message) {
    transport_ok_=message->data;
    transport_received_=Steady::now();
    if (!transport_ok_) setStatus("LiDAR transport reports no points", true);
  }
  static bool layoutValid(const sensor_msgs::PointCloud2& msg) {
    if (msg.is_bigendian || msg.point_step==0) return false;
    if (uint64_t(msg.row_step)<uint64_t(msg.width)*msg.point_step ||
        uint64_t(msg.data.size())!=uint64_t(msg.row_step)*msg.height) return false;
    for (const std::string name : {"x", "y", "z"}) {
      unsigned matches=0;
      for (const auto& field : msg.fields)
        if (field.name==name && field.datatype==sensor_msgs::PointField::FLOAT32 &&
            field.count==1 && uint64_t(field.offset)+4<=msg.point_step) ++matches;
      if (matches!=1) return false;
    }
    return true;
  }
  void scan(const sensor_msgs::PointCloud2::ConstPtr& message) {
    const auto start=Steady::now();
    const auto now=ros::Time::now();
    if (!last_clock_.isZero() && now<last_clock_) {last_stamp_=ros::Time(); transport_ok_=false;}
    last_clock_=now;
    if (message->header.frame_id!="lidar_link") {
      ++health_.invalid_count;
      setStatus("wrong LiDAR frame; no coordinate relabeling allowed", true);
      return;
    }
    if (message->header.stamp.isZero() || message->header.stamp>now ||
        (!last_stamp_.isZero() && message->header.stamp<=last_stamp_)) {
      ++health_.invalid_count;
      setStatus("zero/future/duplicate/regressing scan stamp", true);
      return;
    }
    last_stamp_=message->header.stamp;
    last_received_=start;
    common_msgs_pkg::LidarObservationArray output;
    output.header=message->header;
    output.calibration_id=calibration_id_;
    output.timestamp_provenance="ingress_fallback";
    // These are intentionally false until central live validation is completed.
    output.calibration_verified=false;
    output.freshness_verified=false;
    try {
      if (message->header.frame_id!="lidar_link") throw std::runtime_error("wrong LiDAR frame");
      if (!transport_ok_ || elapsed(transport_received_)>watchdog_sec_)
        throw std::runtime_error("missing/stale/false transport status");
      if (max_scan_age_sec_>0 && (now-message->header.stamp).toSec()>max_scan_age_sec_)
        throw std::runtime_error("scan exceeds separately configured data-age limit");
      if (!layoutValid(*message)) throw std::runtime_error("invalid PointCloud2 XYZ layout");
      if (uint64_t(message->width)*message->height==0)
        throw std::runtime_error("empty raw scan is not evidence of free space");
      lidar_perception::Cloud cloud;
      pcl::fromROSMsg(*message, cloud);
      bool any_finite=false;
      for (const auto& p : cloud)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {any_finite=true; break;}
      if (!any_finite) throw std::runtime_error("no finite XYZ points");
      const auto result=lidar_perception::detect(cloud, config_);
      for (const auto& box : result.boxes) {
        common_msgs_pkg::LidarObjectObservation object;
        object.scan_local_id=output.objects.size();
        object.center.x=box.center.x; object.center.y=box.center.y; object.center.z=box.center.z;
        object.size.x=box.size.x; object.size.y=box.size.y; object.size.z=box.size.z;
        object.point_count=box.point_count;
        object.confidence=-1;
        output.objects.push_back(object);
      }
      output.objects_valid=true;
      ++health_.processed_count;
      sensor_msgs::PointCloud2 filtered;
      pcl::toROSMsg(*result.filtered, filtered);
      filtered.header=message->header;
      debug_.publish(filtered);
      health_.reason="development object geometry only; calibration/freshness unverified; no ground/free-space/velocity";
    } catch (const std::exception& error) {
      output.objects.clear(); output.objects_valid=false;
      ++health_.invalid_count;
      health_.reason=error.what();
    }
    health_.processing_latency_sec=elapsed(start);
    if (output.objects_valid &&
        ((max_scan_age_sec_>0 && (ros::Time::now()-message->header.stamp).toSec()>max_scan_age_sec_) ||
         ros::Time::now()<message->header.stamp)) {
      output.objects.clear(); output.objects_valid=false;
      ++health_.dropped_count;
      health_.reason="scan expired or clock reset during processing";
    }
    observations_.publish(output);
    setStatus(health_.reason, !output.objects_valid);
  }
public:
  Node() {
    private_.param("x_min",config_.x_min,config_.x_min); private_.param("x_max",config_.x_max,config_.x_max);
    private_.param("y_min",config_.y_min,config_.y_min); private_.param("y_max",config_.y_max,config_.y_max);
    private_.param("z_min",config_.z_min,config_.z_min); private_.param("z_max",config_.z_max,config_.z_max);
    private_.param("leaf_size",config_.leaf_size,config_.leaf_size);
    private_.param("epsilon",config_.epsilon,config_.epsilon);
    private_.param("min_points",config_.min_points,config_.min_points);
    private_.param("min_cluster_size",config_.min_cluster_size,config_.min_cluster_size);
    private_.param("max_cluster_size",config_.max_cluster_size,config_.max_cluster_size);
    config_.validate();
    if (!private_.getParam("contract/development_watchdog_sec",watchdog_sec_) ||
        !std::isfinite(watchdog_sec_) || watchdog_sec_<=0 ||
        !private_.getParam("contract/calibration_id",calibration_id_) || calibration_id_.empty())
      throw std::runtime_error("load central lidar_messages.yaml contract using package launch");
    if (!private_.getParam("contract/max_scan_age_sec",max_scan_age_sec_) ||
        !std::isfinite(max_scan_age_sec_) || max_scan_age_sec_<0 ||
        !private_.getParam("contract/status_period_sec",status_period_sec_) ||
        !std::isfinite(status_period_sec_) || status_period_sec_<=0)
      throw std::runtime_error("invalid central data-age/status period policy");
    health_.processing_latency_sec=-1;
    observations_=nh_.advertise<common_msgs_pkg::LidarObservationArray>("/molit/perception/lidar/observations",2);
    status_=nh_.advertise<common_msgs_pkg::ComponentStatus>("/molit/perception/lidar/status",1,true);
    debug_=private_.advertise<sensor_msgs::PointCloud2>("filtered_points",1);
    points_=nh_.subscribe("/molit/sensors/lidar/points",1,&Node::scan,this);
    transport_=nh_.subscribe("/molit/sensors/lidar/status",1,&Node::transport,this);
    timer_=nh_.createWallTimer(ros::WallDuration(status_period_sec_),&Node::tick,this);
    setStatus("waiting for LiDAR input; development only");
  }
};
}
int main(int argc, char** argv) {
  ros::init(argc,argv,"lidar_perception_node");
  try {Node node; ros::spin();}
  catch (const std::exception& error) {ROS_FATAL("%s",error.what()); return 1;}
  return 0;
}
