cmake_minimum_required(VERSION 3.16...3.31)
project(cx_rl_gym)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()
