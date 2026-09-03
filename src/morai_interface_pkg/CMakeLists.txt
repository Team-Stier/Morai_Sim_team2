cmake_minimum_required(VERSION 3.0.2)
project(morai_interface_pkg)

find_package(catkin REQUIRED COMPONENTS
  common_msgs_pkg
  geometry_msgs
  ros_architecture_pkg
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    common_msgs_pkg
    geometry_msgs
    ros_architecture_pkg
    rospy
    sensor_msgs
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/camera_receiver_node
  scripts/gps_receiver_node
  scripts/imu_receiver_node
  scripts/lidar_watchdog_node
  scripts/vehicle_status_receiver_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config docs launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".gitkeep" EXCLUDE)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_camera_packet.py)
  catkin_add_nosetests(test/test_ego_status_packet.py)
  catkin_add_nosetests(test/test_imu_packet.py)
  catkin_add_nosetests(test/test_nmea_gps.py)
  catkin_add_nosetests(test/test_udp_loopback.py)
  catkin_add_nosetests(test/test_udp_loopback_binary.py)
  catkin_add_nosetests(test/test_udp_loopback_camera.py)
  catkin_add_nosetests(test/test_watchdog_util.py)
  catkin_add_nosetests(test/test_interface_contract_alignment.py)
  catkin_add_nosetests(test/test_timestamp_guard.py)
endif()
