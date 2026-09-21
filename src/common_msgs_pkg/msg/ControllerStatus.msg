# Schema authority: ros_architecture_pkg/config/messages/controller_messages.yaml
uint8 STOP=0
uint8 PURE_PURSUIT=1
uint8 STANLEY=2
uint8 DEGRADED=3
# Evaluation time; frame_id is empty. Readiness is not permission to actuate.
std_msgs/Header header
bool ready
bool stop_required
bool command_valid
uint8 mode
uint32 reset_id
time odometry_stamp
time trajectory_stamp
float64 target_speed_mps
float64 speed_error_mps
# PP: rear reference lateral error; Stanley: front axle directed path error.
float64 cross_track_error_m
float64 heading_error_rad
bool saturated
string reason
