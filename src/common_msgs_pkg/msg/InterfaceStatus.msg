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
# Interface readiness only, never permission to actuate.
bool ready
bool stop_required
# Bitmasks are defined by the producing morai_interface_pkg implementation.
uint32 required_channel_mask
uint32 healthy_channel_mask
uint32 stale_channel_mask
uint32 invalid_channel_mask
# Latest accepted required-channel data stamp; zero means unavailable.
time data_stamp
# Worst required-channel age at header.stamp; -1 means unavailable.
float64 data_age_sec
uint64 received_count
uint64 dropped_count
uint64 invalid_count
string reason
