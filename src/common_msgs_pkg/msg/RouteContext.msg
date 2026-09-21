# Authority: ros_architecture_pkg/config/messages/rddf_planning_messages.yaml
std_msgs/Header header
string map_id
string reference_sha256
common_msgs_pkg/RouteLane[] lanes
common_msgs_pkg/LaneChangeWindow[] lane_changes
geometry_msgs/Point[] checkpoints
uint32 next_checkpoint
string current_lane
float64 progress
geometry_msgs/Point comparison_goal
float64 comparison_goal_s
bool route_complete
# Original solid/center/restricted/unknown boundary polylines; no joins across IDs.
geometry_msgs/Polygon[] forbidden_boundaries
