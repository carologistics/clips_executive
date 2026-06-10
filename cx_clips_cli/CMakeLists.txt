cmake_minimum_required(VERSION 3.16...3.31)
project(cx_clips_cli)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cli
)

#install(DIRECTORY
#  launch
#  params
#  DESTINATION share/${PROJECT_NAME}
#)

ament_package()
