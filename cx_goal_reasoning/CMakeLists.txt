cmake_minimum_required(VERSION 3.16...3.28)
project(cx_goal_reasoning)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(dependencies
  ament_cmake
)

include_directories(include)

install(DIRECTORY clips DESTINATION share/${PROJECT_NAME}/)
ament_package()
