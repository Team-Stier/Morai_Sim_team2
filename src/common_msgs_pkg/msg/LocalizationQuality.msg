std_msgs/Header header

uint8 INITIALIZING=0
uint8 NOMINAL=1
uint8 DEGRADED=2
uint8 INVALID=3

uint8 state
time state_estimate_stamp
float64 position_covariance_trace_m2
float64 gps_age_sec
float64 imu_age_sec
uint32 rejected_gps_count
string detail
