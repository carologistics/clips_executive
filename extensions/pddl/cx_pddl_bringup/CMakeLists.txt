cmake_minimum_required(VERSION 3.16...3.31)
project(cx_pddl_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cx_ros_comm_gen REQUIRED)

install(DIRECTORY params pddl launch clips DESTINATION share/${PROJECT_NAME})
cx_generate_action_bindings("btcpp_ros2_interfaces/action" ExecuteTree)

ament_package()
