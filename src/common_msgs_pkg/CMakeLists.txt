cmake_minimum_required(VERSION 3.0.2)
project(common_msgs_pkg)

find_package(
  catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  ros_architecture_pkg
  std_msgs
)

add_message_files(
  FILES
  ControllerVehicleState.msg
  LeadVehicleState.msg
  PlannedTrajectory.msg
  RawActuatorCommand.msg
  RouteContext.msg
  Team1ControllerStatus.msg
  TrajectoryPoint.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime ros_architecture_pkg std_msgs
)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
  DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".gitkeep" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
