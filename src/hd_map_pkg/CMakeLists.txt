cmake_minimum_required(VERSION 3.0.2)
project(hd_map_pkg)

find_package(catkin REQUIRED COMPONENTS
  common_msgs_pkg
  geometry_msgs
  ros_architecture_pkg
  rospy
  visualization_msgs
)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS
  common_msgs_pkg
  geometry_msgs
  ros_architecture_pkg
  rospy
  visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/hd_map_rviz_node
  scripts/hd_map_tool
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".gitkeep" EXCLUDE)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
