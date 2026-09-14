# Authority: ros_architecture_pkg/config/messages/lidar_messages.yaml
# Local to this scan; never a persistent tracking ID.
uint32 scan_local_id
# Axis-aligned bounding box in the array header frame, metres.
geometry_msgs/Point center
geometry_msgs/Vector3 size
# DBSCAN: voxel cluster points. Learned box: finite raw returns inside OBB.
uint32 point_count
# -1 = uncalibrated; otherwise a calibrated probability in [0,1].
float32 confidence
uint8 UNKNOWN=0
uint8 PEDESTRIAN=1
uint8 VEHICLE=2
uint8 OTHER=3
# Pedestrian does not infer age or gender. Vehicles exclude two-wheelers.
uint8 semantic_class
# False: clustered returns. True: enclosing AABB of a learned oriented box.
bool learned_box
# Empty/zero for DBSCAN; native nuScenes class and uncalibrated model score.
string model_class
float32 model_score
