cmake_minimum_required(VERSION 3.0.2)
project(runtime_evaluation_pkg)

find_package(catkin REQUIRED COMPONENTS
  common_msgs_pkg
  geometry_msgs
  nav_msgs
  ros_architecture_pkg
  rospy
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    common_msgs_pkg
    geometry_msgs
    nav_msgs
    ros_architecture_pkg
    rospy
    visualization_msgs
)

catkin_install_python(
  PROGRAMS scripts/planning_visualizer_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config docs launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".gitkeep" EXCLUDE)
