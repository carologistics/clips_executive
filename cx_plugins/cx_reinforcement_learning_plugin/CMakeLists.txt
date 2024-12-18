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
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cx_plugin REQUIRED)
find_package(cx_utils REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)
find_package(cx_rl_interfaces REQUIRED)
find_package(cx_ros_comm_gen REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  rclcpp_action
  pluginlib
  cx_plugin
  cx_utils
  cx_rl_interfaces
  clips_vendor
  clips
)
include_directories(include)

pluginlib_export_plugin_description_file(cx_plugin reinforcement_learning_plugin.xml)

add_library(${PROJECT_NAME} SHARED src/reinforcement_learning_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ClipsNS::libclips_ns)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD ${CMAKE_CXX_STANDARD})

cx_generate_srv_bindings("cx_rl_interfaces" "SetRLMode")
cx_generate_srv_bindings("cx_rl_interfaces" "GetDomainObjects")
cx_generate_srv_bindings("cx_rl_interfaces" "GetDomainPredicates")
cx_generate_srv_bindings("cx_rl_interfaces" "GetGoalList")
cx_generate_srv_bindings("cx_rl_interfaces" "GetGoalListRobot")
cx_generate_srv_bindings("cx_rl_interfaces" "CreateRLEnvState")
cx_generate_srv_bindings("cx_rl_interfaces" "ExecGoalSelection")
cx_generate_action_bindings("cx_rl_interfaces" "ResetCX")
cx_generate_action_bindings("cx_rl_interfaces" "GetFreeRobot")
cx_generate_action_bindings("cx_rl_interfaces" "GoalSelection")

install (DIRECTORY clips DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES reinforcement_learning_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
