cmake_minimum_required(VERSION 3.16...3.31)
project(cx_rl_clips)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cx_ros_comm_gen REQUIRED)

set(dependencies
  ament_cmake
  cx_ros_comm_gen
  cx_rl_interfaces
)

cx_generate_action_bindings("cx_rl_interfaces" "ResetEnv")
cx_generate_action_bindings("cx_rl_interfaces" "GetFreeRobot")
cx_generate_action_bindings("cx_rl_interfaces" "ActionSelection")

install (DIRECTORY clips DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
