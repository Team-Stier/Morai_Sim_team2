# Authority: ros_architecture_pkg/config/messages/rddf_planning_messages.yaml
string id
string kind
# Dense occupied XYZ samples, not polygon vertices; never join across samples.
geometry_msgs/Point[] points
