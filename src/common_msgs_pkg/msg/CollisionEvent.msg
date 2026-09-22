# Schema authority: ros_architecture_pkg/config/messages/competition_io_messages.yaml
# MORAI interface ingress time; frame_id is empty because geometry is not exposed here.
std_msgs/Header header
# Source packet sec/nsec when available; zero means unavailable.
time source_stamp
bool collision_detected
uint8 object_count
int16[] object_types
int16[] object_ids
uint32 raw_data_length
