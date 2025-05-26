cmake_minimum_required(VERSION 3.8)
project(cx_reinforcement_learning_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cx_plugin REQUIRED)
find_package(cx_ros_comm_gen REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  rclcpp_action
  pluginlib
  cx_plugin
  cx_rl_interfaces
  clips_vendor
  clips
)

cx_generate_srv_bindings("cx_rl_interfaces" "SetRLMode")
cx_generate_srv_bindings("cx_rl_interfaces" "GetObservableObjects")
cx_generate_srv_bindings("cx_rl_interfaces" "GetObservablePredicates")
cx_generate_srv_bindings("cx_rl_interfaces" "GetPredefinedObservables")
cx_generate_srv_bindings("cx_rl_interfaces" "GetActionList")
cx_generate_srv_bindings("cx_rl_interfaces" "GetActionListRobot")
cx_generate_srv_bindings("cx_rl_interfaces" "CreateRLEnvState")
cx_generate_srv_bindings("cx_rl_interfaces" "ExecActionSelection")
cx_generate_action_bindings("cx_rl_interfaces" "ResetCX")
cx_generate_action_bindings("cx_rl_interfaces" "GetFreeRobot")
cx_generate_action_bindings("cx_rl_interfaces" "ActionSelection")

install (DIRECTORY clips DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
