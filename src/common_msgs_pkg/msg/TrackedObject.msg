# Authority: ros_architecture_pkg/config/messages/world_model_messages.yaml
uint8 CLASS_UNKNOWN=0
uint8 CLASS_PEDESTRIAN=1
uint8 CLASS_VEHICLE=2
uint8 CLASS_OTHER=3
uint8 TRACK_TENTATIVE=0
uint8 TRACK_CONFIRMED=1
uint8 TRACK_COASTING=2
# Persistent only within one world_model_node process; never a Ground Truth ID.
uint32 track_id
# Most recent contributing observation and its original provenance.
time source_stamp
string source_frame_id
uint32 source_local_id
string timestamp_provenance
string calibration_id
bool calibration_verified
# Map-aligned conservative AABB. Orientation is identity until oriented-box fusion exists.
geometry_msgs/Pose pose
geometry_msgs/Vector3 size
# Linear velocity is expressed in map. Angular velocity is not estimated.
geometry_msgs/Twist twist
bool velocity_valid
uint8 semantic_class
float32 confidence
uint8 track_state
uint32 observation_count
# At the enclosing WorldModel fusion_reference_time.
float64 age_since_observation_sec
# -1 means the current development pipeline cannot bound this uncertainty.
float64 position_stddev_m
float64 velocity_stddev_mps
