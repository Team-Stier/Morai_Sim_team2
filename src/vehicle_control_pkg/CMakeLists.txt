cmake_minimum_required(VERSION 3.0.2)
project(vehicle_control_pkg)
add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS common_msgs_pkg ros_architecture_pkg roscpp nav_msgs)
catkin_package(CATKIN_DEPENDS common_msgs_pkg ros_architecture_pkg roscpp nav_msgs)
include_directories(src src/imported ${catkin_INCLUDE_DIRS})
add_library(vehicle_tracking_core
  src/controller.cpp
  src/imported/path_geometry.cpp
  src/imported/pure_pursuit.cpp
  src/imported/stanley.cpp
  src/imported/hybrid_supervisor.cpp)
target_link_libraries(vehicle_tracking_core ${catkin_LIBRARIES})
add_dependencies(vehicle_tracking_core ${catkin_EXPORTED_TARGETS})
add_executable(vehicle_controller_node src/vehicle_controller_node.cpp)
target_link_libraries(vehicle_controller_node vehicle_tracking_core ${catkin_LIBRARIES})
add_dependencies(vehicle_controller_node ${catkin_EXPORTED_TARGETS})
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  foreach(name path_geometry pure_pursuit stanley hybrid_supervisor)
    catkin_add_gtest(test_imported_${name} test/imported/test_${name}.cpp test/main.cpp)
    if(TARGET test_imported_${name})
      target_link_libraries(test_imported_${name} vehicle_tracking_core ${catkin_LIBRARIES})
    endif()
  endforeach()
  if(TARGET test_imported_hybrid_supervisor)
    target_sources(test_imported_hybrid_supervisor PRIVATE test/imported/path_source_manager.cpp)
  endif()
  catkin_add_gtest(test_controller test/test_controller.cpp test/main.cpp)
  if(TARGET test_controller)
    target_link_libraries(test_controller vehicle_tracking_core ${catkin_LIBRARIES})
  endif()
  catkin_add_nosetests(test/test_interface_contract.py)
  catkin_install_python(PROGRAMS test/test_controller_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/controller.test DEPENDENCIES vehicle_controller_node)
endif()
install(TARGETS vehicle_tracking_core vehicle_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config docs launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN ".gitkeep" EXCLUDE)
