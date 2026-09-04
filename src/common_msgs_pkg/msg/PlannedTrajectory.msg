uint8 STATUS_VALID=0
uint8 STATUS_STOP_REQUIRED=1
uint8 STATUS_INPUT_STALE=2
uint8 STATUS_NO_PATH=3
uint8 STATUS_INVALID_START=4

std_msgs/Header header
time valid_until
uint8 status
string reason
bool lane_change_authorized
float32 minimum_boundary_clearance_m
common_msgs_pkg/TrajectoryPoint[] points
