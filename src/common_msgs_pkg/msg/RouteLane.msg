# Authority: ros_architecture_pkg/config/messages/rddf_planning_messages.yaml
string id
string link_id
nav_msgs/Path centerline
string[] successors
uint32[] source_indices
float64[] route_s
# -1 means the course speed limit is lifted; not a cruise target.
float32[] speed_limits_mps
