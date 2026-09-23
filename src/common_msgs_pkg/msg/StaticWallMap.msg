# Authority: ros_architecture_pkg/config/messages/static_wall_messages.yaml
# Immutable map version load time, frame_id=map (not a sensor timestamp).
std_msgs/Header header
string map_id
string source_sha256
# Parallel arrays; each baseline is an OPEN original wall polyline in map metres.
string[] wall_ids
geometry_msgs/Polygon[] baselines
# Conservative horizontal model uncertainty, not measured physical accuracy.
float64 horizontal_stddev_m
bool physical_alignment_verified
