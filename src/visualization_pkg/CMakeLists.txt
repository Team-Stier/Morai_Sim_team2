cmake_minimum_required(VERSION 3.0.2)
project(visualization_pkg)

find_package(catkin REQUIRED COMPONENTS
  hd_map_pkg ros_architecture_pkg common_msgs_pkg geometry_msgs nav_msgs rospy visualization_msgs)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS
  hd_map_pkg ros_architecture_pkg common_msgs_pkg geometry_msgs nav_msgs rospy visualization_msgs)

catkin_install_python(PROGRAMS src/vehicle_visualizer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_interface_contract.py)
  catkin_add_nosetests(test/test_vehicle_display.py)
  catkin_add_nosetests(test/test_hd_map_display.py)
  add_rostest(test/vehicle_visualizer.test)
endif()

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".gitkeep" EXCLUDE)
