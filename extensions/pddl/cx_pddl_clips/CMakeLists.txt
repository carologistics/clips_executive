cmake_minimum_required(VERSION 3.16...3.28)
project(cx_pddl_clips)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install (DIRECTORY clips DESTINATION share/${PROJECT_NAME})

ament_package()
