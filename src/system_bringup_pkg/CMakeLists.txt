cmake_minimum_required(VERSION 3.0.2)
project(system_bringup_pkg)

find_package(catkin REQUIRED COMPONENTS
  camera_perception_pkg
  common_msgs_pkg
  global_route_manager_pkg
  hd_map_pkg
  lidar_perception_pkg
  localization_pkg
  morai_interface_pkg
  path_planning_pkg
  ros_architecture_pkg
  rospy
  runtime_evaluation_pkg
  safety_supervisor_pkg
  vehicle_control_pkg
  world_model_pkg
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    camera_perception_pkg
    common_msgs_pkg
    global_route_manager_pkg
    hd_map_pkg
    lidar_perception_pkg
    localization_pkg
    morai_interface_pkg
    path_planning_pkg
    ros_architecture_pkg
    rospy
    runtime_evaluation_pkg
    safety_supervisor_pkg
    vehicle_control_pkg
    world_model_pkg
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_live_morai_sensor_ingress.py)
  catkin_add_nosetests(test/test_full_stack_launch_skeleton.py)
  catkin_add_nosetests(test/test_readiness_logic.py)
  catkin_add_nosetests(test/test_system_readiness_contract.py)
endif()

catkin_install_python(
  PROGRAMS scripts/system_readiness_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
  DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".gitkeep" EXCLUDE
)
