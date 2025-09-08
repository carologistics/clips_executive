cmake_minimum_required(VERSION 3.16...3.28)
project(cx_pddl_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cx_ros_comm_gen REQUIRED)

install(DIRECTORY params pddl launch clips DESTINATION share/${PROJECT_NAME})

cx_generate_msg_bindings("cx_pddl_msgs" "TimedPlanAction")
cx_generate_action_bindings("cx_pddl_msgs" "PlanTemporal")

ament_package()
