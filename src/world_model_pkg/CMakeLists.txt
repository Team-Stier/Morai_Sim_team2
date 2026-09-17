cmake_minimum_required(VERSION 3.0.2)
project(world_model_pkg)

find_package(catkin REQUIRED COMPONENTS
  common_msgs_pkg
  geometry_msgs
  rospy
  ros_architecture_pkg
  tf2_ros
)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS
  common_msgs_pkg
  geometry_msgs
  rospy
  ros_architecture_pkg
  tf2_ros
)

catkin_install_python(PROGRAMS scripts/world_model_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_tracking.py)
  catkin_add_nosetests(test/test_transforms.py)
  catkin_add_nosetests(test/test_interface_contract.py)
  add_rostest(test/world_model.test)
endif()

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config docs launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".gitkeep" EXCLUDE)
