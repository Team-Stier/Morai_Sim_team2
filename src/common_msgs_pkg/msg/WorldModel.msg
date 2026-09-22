# Authority: ros_architecture_pkg/config/messages/world_model_messages.yaml
# Fusion reference time; frame_id must be map.
std_msgs/Header header
# Localization epoch used for every scan-time transform in this scene.
uint32 localization_reset_id
# True means the listed object geometry was transformed and tracked successfully.
bool objects_valid
bool tracking_valid
# True only when every driving-required calibration/freshness/uncertainty gate passes.
bool objects_verified
# Capabilities absent from the first LiDAR-only vertical slice remain false.
bool map_context_valid
bool route_context_valid
bool occupancy_valid
bool free_space_valid
bool traffic_context_valid
# Consumers must not plan from this message unless planner_ready is true.
bool planner_ready
common_msgs_pkg/TrackedObject[] objects
string reason
