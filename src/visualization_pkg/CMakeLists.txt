cmake_minimum_required(VERSION 3.0.2)
project(visualization_pkg)

find_package(catkin REQUIRED COMPONENTS
  hd_map_pkg ros_architecture_pkg common_msgs_pkg geometry_msgs nav_msgs rospy roscpp rviz pluginlib tf2_ros visualization_msgs)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS
  hd_map_pkg ros_architecture_pkg common_msgs_pkg geometry_msgs nav_msgs rospy roscpp rviz pluginlib tf2_ros visualization_msgs
  LIBRARIES planner_status_display)

add_library(planner_status_display src/planner_status_display.cpp)
target_compile_features(planner_status_display PRIVATE cxx_std_14)
target_include_directories(planner_status_display PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(planner_status_display ${catkin_LIBRARIES} Qt5::Widgets)
add_dependencies(planner_status_display ${catkin_EXPORTED_TARGETS})
install(TARGETS planner_status_display LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES rviz_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS src/vehicle_visualizer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_planner_status_text test/test_planner_status_text.cpp)
  if(TARGET test_planner_status_text)
    target_include_directories(test_planner_status_text PRIVATE src)
    target_compile_features(test_planner_status_text PRIVATE cxx_std_14)
  endif()
  catkin_add_nosetests(test/test_interface_contract.py)
  catkin_add_nosetests(test/test_vehicle_display.py)
  catkin_add_nosetests(test/test_lidar_display.py)
  catkin_add_nosetests(test/test_world_model_display.py)
  catkin_add_nosetests(test/test_hd_map_display.py)
  add_rostest(test/vehicle_visualizer.test)
  add_rostest(test/lidar_pipeline.test)
endif()

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".gitkeep" EXCLUDE)
