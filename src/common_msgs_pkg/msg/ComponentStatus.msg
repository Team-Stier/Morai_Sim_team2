# Schema authority: ros_architecture_pkg/config/messages/core_messages.yaml
uint8 UNKNOWN=0
uint8 INITIALIZING=1
uint8 READY=2
uint8 DEGRADED=3
uint8 FAULT=4
uint8 DISABLED=5
# Evaluation time in the active canonical clock; frame_id is empty.
std_msgs/Header header
string component
uint8 state
# Component readiness only, never permission to actuate.
bool ready
bool stop_required
# Latest evaluated data stamp; zero means no data. Static data uses load time.
time data_stamp
# Seconds at header.stamp; -1 means unavailable, not zero-age.
float64 data_age_sec
# Processing duration from a monotonic clock; -1 means unavailable.
float64 processing_latency_sec
# Counters since process startup, not necessarily network packet counters.
uint64 processed_count
uint64 dropped_count
uint64 invalid_count
# Human-readable detail; consumers must use state/ready/stop_required.
string reason
