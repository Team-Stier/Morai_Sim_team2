# Schema authority: ros_architecture_pkg/config/messages/readiness_messages.yaml
uint8 UNKNOWN=0
uint8 INITIALIZING=1
uint8 READY=2
uint8 DEGRADED=3
uint8 FAULT=4
# Stable component-mask bits owned by system_bringup_pkg.
uint32 COMPONENT_INTERFACE=1
uint32 COMPONENT_MAP=2
uint32 COMPONENT_CAMERA_PERCEPTION=4
uint32 COMPONENT_LIDAR_PERCEPTION=8
uint32 COMPONENT_LOCALIZATION=16
uint32 COMPONENT_ROUTE=32
uint32 COMPONENT_WORLD_MODEL=64
uint32 COMPONENT_PLANNING=128
uint32 COMPONENT_CONTROL=256
# Evaluation time in the active canonical clock; frame_id is empty.
std_msgs/Header header
uint8 state
# Upstream readiness only. Safety remains the final driving authority.
bool ready
uint32 required_mask
uint32 ready_mask
uint32 missing_mask
uint32 stale_mask
uint32 fault_mask
# Maximum nonnegative age among received required statuses; -1 means none received.
float64 max_status_age_sec
string reason
