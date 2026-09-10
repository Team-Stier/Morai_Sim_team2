# Schema authority: ros_architecture_pkg/config/messages/readiness_messages.yaml
uint8 UNKNOWN=0
uint8 INITIALIZING=1
uint8 READY=2
uint8 DEGRADED=3
uint8 FAULT=4
uint8 DISABLED=5
# Evaluation time in the active canonical clock; frame_id is empty.
std_msgs/Header header
uint8 state
# Controller readiness only, never permission to actuate.
bool ready
bool stop_required
bool trajectory_valid
bool nominal_command_valid
# Source stamps preserved by the controller; zero means unavailable.
time trajectory_stamp
time nominal_command_stamp
# Ages at header.stamp; -1 means unavailable.
float64 trajectory_age_sec
float64 nominal_command_age_sec
# Monotonic processing duration; -1 means unavailable.
float64 processing_latency_sec
string reason
