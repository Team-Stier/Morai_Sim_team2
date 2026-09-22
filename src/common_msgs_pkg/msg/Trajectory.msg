# Schema authority: ros_architecture_pkg/config/messages/controller_messages.yaml
# Planning reference time; frame_id must be odom. Forward motion only.
std_msgs/Header header
uint32 reset_id
bool valid
bool stop_required
# Valid after header.stamp and strictly before header.stamp + valid_for.
duration valid_for
# Equal-sized ordered arrays; time starts at zero and strictly increases.
geometry_msgs/Pose[] poses
float64[] speed_mps
duration[] time_from_start
