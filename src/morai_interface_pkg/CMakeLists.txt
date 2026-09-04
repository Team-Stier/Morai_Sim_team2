cmake_minimum_required(VERSION 3.0.2)
project(morai_interface_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  morai_msgs
)

# src/의 파이썬 모듈(morai_interface_pkg.*)을 노출한다.
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs morai_msgs
)

# rosrun/roslaunch로 실행하는 파이썬 진입점 설치.
catkin_install_python(PROGRAMS
  scripts/gps_receiver_node
  scripts/imu_receiver_node
  scripts/vehicle_status_receiver_node
  scripts/camera_receiver_node
  scripts/lidar_watchdog_node
  scripts/ctrl_cmd_sender_node
  scripts/fake_gps_udp_sender
  scripts/fake_imu_udp_sender
  scripts/fake_vehicle_status_udp_sender
  scripts/fake_camera_udp_sender
  scripts/fake_ctrl_cmd_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_nmea_gps.py)
  catkin_add_nosetests(test/test_imu_packet.py)
  catkin_add_nosetests(test/test_ego_status_packet.py)
  catkin_add_nosetests(test/test_camera_packet.py)
  catkin_add_nosetests(test/test_ego_ctrl_cmd_packet.py)
  catkin_add_nosetests(test/test_watchdog_util.py)
  catkin_add_nosetests(test/test_udp_loopback.py)
  catkin_add_nosetests(test/test_udp_loopback_binary.py)
  catkin_add_nosetests(test/test_udp_loopback_camera.py)
  catkin_add_nosetests(test/test_udp_loopback_ctrl.py)
endif()
