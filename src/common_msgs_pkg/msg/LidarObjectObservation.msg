# Authority: ros_architecture_pkg/config/messages/lidar_messages.yaml
# Local to this scan; never a persistent tracking ID.
uint32 scan_local_id
# Actual clustered points in the array header frame, metres; no bounding box.
geometry_msgs/Point[] points
# Original returns assigned to accepted DBSCAN clusters, without resampling.
uint32 point_count
# -1 = uncalibrated; otherwise a calibrated probability in [0,1].
float32 confidence
