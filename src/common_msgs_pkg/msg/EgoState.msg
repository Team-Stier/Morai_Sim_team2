# Schema authority: ros_architecture_pkg/config/messages/core_messages.yaml
# Estimated state, NEVER a Competition Vehicle Status ground-truth pose.
# Estimate validity time; frame_id must be map.
std_msgs/Header header
# Pose refers to base_link; twist is expressed in base_link.
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
# Valid estimated components [x,y,z,roll,pitch,yaw], NOT sensor availability.
bool[6] pose_valid
# Valid estimated components [vx,vy,vz,wx,wy,wz], NOT raw measurements.
bool[6] twist_valid
# Changes on localization discontinuity/reset; consumers flush temporal buffers.
uint32 reset_id
