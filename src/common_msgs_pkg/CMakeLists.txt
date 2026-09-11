cmake_minimum_required(VERSION 3.0.2)
project(common_msgs_pkg)

find_package(catkin REQUIRED COMPONENTS
  ros_architecture_pkg message_generation std_msgs geometry_msgs)

catkin_python_setup()

add_message_files(FILES ComponentStatus.msg EgoState.msg LocalizationStatus.msg
  LidarObjectObservation.msg LidarObservationArray.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS
  ros_architecture_pkg message_runtime std_msgs geometry_msgs)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_core_contract.py)
  catkin_add_nosetests(test/test_validation.py)
  catkin_add_nosetests(test/test_serialization.py)
  catkin_add_nosetests(test/test_lidar_contract.py)
endif()

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
  DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".gitkeep" EXCLUDE
)
