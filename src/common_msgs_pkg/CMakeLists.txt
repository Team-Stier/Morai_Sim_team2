cmake_minimum_required(VERSION 3.0.2)
project(common_msgs_pkg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ros_architecture_pkg
  std_msgs
)

add_message_files(
  FILES
  LocalizationQuality.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime ros_architecture_pkg std_msgs
)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
  DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".gitkeep" EXCLUDE
)
