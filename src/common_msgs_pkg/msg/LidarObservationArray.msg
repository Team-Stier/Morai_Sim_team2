# Authority: ros_architecture_pkg/config/messages/lidar_messages.yaml
# Original input scan stamp; frame_id = lidar_link.
std_msgs/Header header
string calibration_id
bool calibration_verified
string timestamp_provenance
bool freshness_verified
# Successful geometry extraction; does not mean safe or complete free space.
bool objects_valid
common_msgs_pkg/LidarObjectObservation[] objects
# Capabilities absent from this object-only baseline must remain false.
bool ground_valid
bool free_space_valid
bool occupancy_valid
bool velocity_valid
