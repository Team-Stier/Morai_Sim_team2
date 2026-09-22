# Authority: ros_architecture_pkg/config/messages/rddf_planning_messages.yaml
std_msgs/Header header
string map_id
string reference_sha256
# Static geometry is cached from HdMap with this map_id.
uint32 next_checkpoint
# next_checkpoint=UINT32_MAX in RDDF-only mode; comparison goal is a forward route station.
string current_lane
float64 progress
geometry_msgs/Point comparison_goal
float64 comparison_goal_s
bool route_complete
