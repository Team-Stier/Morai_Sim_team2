cmake_minimum_required(VERSION 3.0.2)
project(localization_pkg)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  robot_localization
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_path(GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/UTMUPS.hpp REQUIRED)
find_library(GEOGRAPHICLIB_LIBRARY NAMES Geographic REQUIRED)
find_package(Threads REQUIRED)
set(GeographicLib_INCLUDE_DIRS ${GEOGRAPHICLIB_INCLUDE_DIR})
set(GeographicLib_LIBRARIES ${GEOGRAPHICLIB_LIBRARY})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES localization_gps_core
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs nav_msgs robot_localization
                 roscpp rospy sensor_msgs std_msgs
  DEPENDS GeographicLib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GEOGRAPHICLIB_INCLUDE_DIR}
)

add_library(localization_gps_core
  src/freshness.cpp
  src/gps_projection.cpp
  src/gps_gate.cpp
  src/localization_health.cpp
)
target_link_libraries(localization_gps_core ${GEOGRAPHICLIB_LIBRARY})

add_executable(gps_projector_node src/gps_projector_node.cpp)
target_link_libraries(gps_projector_node
  localization_gps_core
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(localization_supervisor_node
  src/localization_supervisor_node.cpp
)
target_link_libraries(localization_supervisor_node
  localization_gps_core
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS localization_gps_core gps_projector_node localization_supervisor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_configuration.py)
  catkin_add_gtest(test_gps_projection test/test_gps_projection.cpp)
  target_link_libraries(test_gps_projection localization_gps_core)
  catkin_add_gtest(test_gps_gate test/test_gps_gate.cpp)
  target_link_libraries(test_gps_gate localization_gps_core)
  catkin_add_gtest(test_freshness test/test_freshness.cpp)
  target_link_libraries(test_freshness localization_gps_core)
  catkin_add_gtest(test_localization_health test/test_localization_health.cpp)
  target_link_libraries(test_localization_health localization_gps_core)
  add_rostest(test/gps_projector.test)
  add_rostest(test/localization_integration.test)
endif()
