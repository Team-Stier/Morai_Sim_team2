cmake_minimum_required(VERSION 3.0.2)
project(lidar_perception_pkg)

add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS common_msgs_pkg ros_architecture_pkg
  roscpp rospy sensor_msgs std_msgs pcl_conversions pcl_ros tf2_ros)
catkin_python_setup()
find_package(PCL REQUIRED COMPONENTS common filters search kdtree)
catkin_package(CATKIN_DEPENDS common_msgs_pkg ros_architecture_pkg
  roscpp rospy sensor_msgs std_msgs pcl_conversions pcl_ros tf2_ros)
include_directories(src ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(lidar_detector src/detector.cpp src/horizontalization.cpp src/cluster_points.cpp src/self_filter.cpp)
target_link_libraries(lidar_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(lidar_perception_node src/lidar_perception_node.cpp)
add_dependencies(lidar_perception_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_perception_node lidar_detector ${catkin_LIBRARIES})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_lidar_detector test/test_detector.cpp test/test_horizontalization.cpp)
  if(TARGET test_lidar_detector)
    target_link_libraries(test_lidar_detector lidar_detector ${catkin_LIBRARIES})
  endif()
  find_package(rostest REQUIRED)
  add_rostest(test/detection.test)
  add_rostest(test/horizontalization.test)
  add_rostest(test/learned.test)
  catkin_add_nosetests(test/test_learned.py)
  catkin_add_nosetests(test/test_horizontalization_metrics.py)
endif()
catkin_install_python(PROGRAMS test/test_node.py test/test_horizontalization_node.py test/test_learned_node.py src/learned_lidar_node.py src/measure_horizontalization.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS lidar_detector lidar_perception_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config docs launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".gitkeep" EXCLUDE)
