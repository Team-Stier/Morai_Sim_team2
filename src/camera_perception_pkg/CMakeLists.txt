cmake_minimum_required(VERSION 3.0.2)
project(camera_perception_pkg)

find_package(catkin REQUIRED COMPONENTS common_msgs_pkg ros_architecture_pkg rospy sensor_msgs cv_bridge)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS common_msgs_pkg ros_architecture_pkg rospy sensor_msgs cv_bridge)
catkin_install_python(PROGRAMS scripts/camera_perception_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_semantic_contract.py)
endif()

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config docs launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".gitkeep" EXCLUDE)
