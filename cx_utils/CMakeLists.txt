cmake_minimum_required(VERSION 3.16...3.28)
project(cx_utils)

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
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  spdlog
  fmt
  ament_index_cpp
)

include_directories(include)

set(CX_UTILS_SOURCES
  src/clips_env_context.cpp
  src/param_utils.cpp
)

add_library(${PROJECT_NAME} SHARED ${CX_UTILS_SOURCES})
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies})
target_link_libraries(${PROJECT_NAME} PUBLIC spdlog::spdlog ClipsNS::libclips_ns)
target_link_libraries(${PROJECT_NAME} INTERFACE spdlog::spdlog ClipsNS::libclips_ns)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
