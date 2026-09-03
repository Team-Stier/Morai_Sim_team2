cmake_minimum_required(VERSION 3.0.2)
project(ros_architecture_pkg)

find_package(catkin REQUIRED)

catkin_package()

install(
  FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
