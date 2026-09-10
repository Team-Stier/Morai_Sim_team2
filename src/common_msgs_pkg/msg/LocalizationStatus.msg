# Schema authority: ros_architecture_pkg/config/messages/core_messages.yaml
uint8 UNINITIALIZED=0
uint8 INITIALIZING=1
uint8 TRACKING=2
uint8 DEAD_RECKONING=3
uint8 RELOCALIZING=4
uint8 LOST=5
# Evaluation time; frame_id is empty.
std_msgs/Header header
uint8 mode
# GPS availability is separate from map/local estimate quality.
bool gps_fix_valid
bool map_pose_valid
bool local_odometry_valid
bool stop_required
# Stamps of the evaluated map and local estimates; zero means absent.
time ego_state_stamp
time local_odometry_stamp
# Same reset epoch as EgoState.
uint32 reset_id
# Age of most recent accepted GPS fix at header.stamp; -1 means never accepted.
float64 gps_age_sec
# Modelled uncertainty, NOT simulator noise settings; -1 means unavailable.
float64 map_position_stddev_m
float64 local_position_stddev_m
float64 yaw_stddev_rad
string reason
