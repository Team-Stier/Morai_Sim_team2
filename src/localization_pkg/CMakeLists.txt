cmake_minimum_required(VERSION 3.0.2)
project(localization_pkg)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  common_msgs_pkg
  geometry_msgs
  nav_msgs
  ros_architecture_pkg
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS common_msgs_pkg geometry_msgs nav_msgs ros_architecture_pkg roscpp
                 sensor_msgs
  DEPENDS Eigen3
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(localization_ekf src/localization_ekf.cpp)
target_link_libraries(localization_ekf ${catkin_LIBRARIES})

# Legacy ROS adapter is retained as source only; its public contract is obsolete.
# Build only the ROS-independent EKF until an approved adapter is implemented.

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_localization_ekf test/test_localization_ekf.cpp)
  if(TARGET test_localization_ekf)
    target_link_libraries(test_localization_ekf localization_ekf ${catkin_LIBRARIES})
  endif()
  catkin_add_nosetests(test/test_interface_contract.py)
endif()

install(TARGETS localization_ekf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
  DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".gitkeep" EXCLUDE
)
