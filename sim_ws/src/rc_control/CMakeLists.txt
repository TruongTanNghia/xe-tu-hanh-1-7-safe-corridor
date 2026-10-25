cmake_minimum_required(VERSION 3.8)
project(rc_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS scripts/ackermann_cmd_bridge.py
  scripts/pid_baseline_controller.py
  DESTINATION lib/${PROJECT_NAME}/
)


ament_export_dependencies(
  controller_manager
  joint_state_broadcaster
  forward_command_controller
  position_controllers
  velocity_controllers
  rclpy
  geometry_msgs
  std_msgs
)

ament_package()
