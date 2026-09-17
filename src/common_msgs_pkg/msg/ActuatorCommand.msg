# Schema authority: ros_architecture_pkg/config/messages/competition_io_messages.yaml
uint8 GEAR_MANUAL=0
uint8 GEAR_PARK=1
uint8 GEAR_REVERSE=2
uint8 GEAR_NEUTRAL=3
uint8 GEAR_DRIVE=4
uint8 GEAR_LOW=5
# command_generation_time; frame_id must be base_link.
std_msgs/Header header
bool valid
# ROS-domain steering angle. Do not equate this to MORAI normalized steer without calibration.
float64 steering_rad
float64 accel
float64 brake
uint8 gear
