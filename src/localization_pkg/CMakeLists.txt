cmake_minimum_required(VERSION 3.0.2)
project(localization_pkg)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  common_msgs_pkg
  geometry_msgs
  nav_msgs
  ros_architecture_pkg
  roscpp
  rospy
  tf2_ros
  roslib
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS common_msgs_pkg geometry_msgs nav_msgs ros_architecture_pkg roscpp
                 sensor_msgs rospy tf2_ros
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(localization_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_interface_contract.py)
  add_rostest(test/localization_node_diagnostic.test)
  add_rostest(test/localization_estimator.test)
  add_rostest(test/wall_matching.test)
  catkin_add_nosetests(test/test_wall_matching.py)
  catkin_add_nosetests(test/test_wall_validity.py)
  catkin_add_nosetests(test/test_live_estimator_core.py)
  catkin_add_nosetests(test/test_blackout_drift.py)
endif()

install(TARGETS localization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
  DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".gitkeep" EXCLUDE
)

catkin_install_python(PROGRAMS src/localization_estimator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
