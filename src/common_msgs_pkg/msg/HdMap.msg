# Authority: ros_architecture_pkg/config/messages/rddf_planning_messages.yaml
std_msgs/Header header
string map_id
string reference_sha256
common_msgs_pkg/RouteLane[] lanes
common_msgs_pkg/LaneChangeWindow[] lane_changes
geometry_msgs/Point[] checkpoints
float64 checkpoint_radius_m
# Original solid/center/restricted/unknown boundary polylines; no joins across IDs.
geometry_msgs/Polygon[] forbidden_boundaries
