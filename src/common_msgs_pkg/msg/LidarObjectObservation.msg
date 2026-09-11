# Authority: ros_architecture_pkg/config/messages/lidar_messages.yaml
# Local to this scan; never a persistent tracking ID.
uint32 scan_local_id
# Axis-aligned bounding box in the array header frame, metres.
geometry_msgs/Point center
geometry_msgs/Vector3 size
# Points after ROI and VoxelGrid, not raw returns.
uint32 point_count
# -1 = uncalibrated; otherwise a calibrated probability in [0,1].
float32 confidence
