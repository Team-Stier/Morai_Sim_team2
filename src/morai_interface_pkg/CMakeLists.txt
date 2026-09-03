cmake_minimum_required(VERSION 3.0.2)
project(morai_interface_pkg)

find_package(catkin REQUIRED COMPONENTS common_msgs_pkg ros_architecture_pkg)
catkin_package(CATKIN_DEPENDS common_msgs_pkg ros_architecture_pkg)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config docs launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".gitkeep" EXCLUDE)
