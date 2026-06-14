cmake_minimum_required(VERSION 3.16...3.31)
project(cx_rl_multi_robot_mppo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/cx_rl_mppo_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cx_rl_mppo_node
)

ament_package()
