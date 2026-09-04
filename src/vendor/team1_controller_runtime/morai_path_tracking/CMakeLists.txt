cmake_minimum_required(VERSION 3.0.2)
project(morai_path_tracking)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  common_msgs_pkg
  geometry_msgs
  message_filters
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES morai_pure_pursuit morai_longitudinal_pid morai_control_timing
            morai_curvature_speed_planner morai_stanley_controller
            morai_imm_two_model_filter morai_hybrid_controller
            morai_wheel_corridor
  CATKIN_DEPENDS common_msgs_pkg geometry_msgs message_filters
                 nav_msgs roscpp std_msgs tf2
                 tf2_geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(morai_pure_pursuit src/controllers/lateral/pure_pursuit.cpp)
target_compile_features(morai_pure_pursuit PUBLIC cxx_std_14)

add_library(morai_longitudinal_pid src/controllers/longitudinal/pid_controller.cpp)
target_compile_features(morai_longitudinal_pid PUBLIC cxx_std_14)

add_library(morai_control_timing src/common/control_timing_bounds.cpp)
target_compile_features(morai_control_timing PUBLIC cxx_std_14)

add_library(morai_curvature_speed_planner src/planning/curvature_speed_planner.cpp)
target_compile_features(morai_curvature_speed_planner PUBLIC cxx_std_14)

add_library(morai_wheel_corridor src/planning/wheel_corridor.cpp)
target_compile_features(morai_wheel_corridor PUBLIC cxx_std_14)

add_library(morai_stanley_controller src/controllers/lateral/stanley_controller.cpp)
target_compile_features(morai_stanley_controller PUBLIC cxx_std_14)

add_library(morai_imm_two_model_filter src/controllers/lateral/imm_two_model_filter.cpp)
target_compile_features(morai_imm_two_model_filter PUBLIC cxx_std_14)

add_library(morai_hybrid_controller src/controllers/lateral/hybrid_controller.cpp)
target_compile_features(morai_hybrid_controller PUBLIC cxx_std_14)
target_link_libraries(morai_hybrid_controller
  morai_pure_pursuit
  morai_stanley_controller
  morai_imm_two_model_filter
)

add_executable(path_tracking_controller_node src/nodes/path_tracking_controller_node.cpp)
target_compile_features(path_tracking_controller_node PUBLIC cxx_std_14)
target_link_libraries(path_tracking_controller_node
  morai_pure_pursuit
  morai_longitudinal_pid
  morai_control_timing
  morai_curvature_speed_planner
  morai_wheel_corridor
  morai_stanley_controller
  morai_hybrid_controller
  ${catkin_LIBRARIES}
)
add_dependencies(path_tracking_controller_node
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS morai_pure_pursuit morai_longitudinal_pid morai_control_timing
          morai_curvature_speed_planner morai_stanley_controller
          morai_imm_two_model_filter morai_hybrid_controller
          morai_wheel_corridor path_tracking_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
