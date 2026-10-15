# Authority: ros_architecture_pkg/config/messages/traffic_signal_messages.yaml
# Camera: source measurement header, empty map IDs. World: same stamp, map frame and selected IDs.
std_msgs/Header header
uint8 RED=1
uint8 YELLOW=2
uint8 GREEN=4
uint8 LEFT=8
uint8 lamp_mask
bool valid
uint8 layout
float32 confidence
float32[4] bbox
string stop_line_id
string[] signal_ids
geometry_msgs/Point position
string reason
