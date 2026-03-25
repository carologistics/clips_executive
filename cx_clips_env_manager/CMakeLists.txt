cmake_minimum_required(VERSION 3.16...3.31)
project(cx_clips_env_manager)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies of env manager
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(cx_msgs REQUIRED)
find_package(cx_utils REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)
find_package(bond REQUIRED)
find_package(bondcpp REQUIRED)

# additional dependencies of plugin manager
find_package(cx_plugin REQUIRED)
find_package(pluginlib REQUIRED)

# -----------------------------
# Sources
# -----------------------------
set(cx_env_manager_sources
  src/clips_env_manager.cpp
)

set(cx_plugin_manager_sources
  src/clips_plugin_manager.cpp
)

# -----------------------------
# Plugin manager library
# -----------------------------
add_library(cx_plugin_manager SHARED ${cx_plugin_manager_sources})

target_include_directories(cx_plugin_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cx_plugin_manager PUBLIC
  ClipsNS::libclips_ns
  rclcpp::rclcpp
  cx_plugin::cx_plugin
  cx_utils::cx_utils
  ${cx_msgs_TARGETS}
  pluginlib::pluginlib
)

set_property(TARGET cx_plugin_manager PROPERTY CXX_STANDARD 20)

# -----------------------------
# Env manager library
# -----------------------------
add_library(${PROJECT_NAME} SHARED ${cx_env_manager_sources})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ClipsNS::libclips_ns
  cx_plugin_manager
  bondcpp::bondcpp
  ${bond_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  ${lifecycle_msgs_TARGETS}
  ${cx_msgs_TARGETS}
  cx_utils::cx_utils
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)

# -----------------------------
# Install rules
# -----------------------------
install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  cx_plugin_manager
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# Exports
# -----------------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  ament_cmake
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  cx_msgs
  cx_utils
  cx_plugin
  pluginlib
  clips_vendor
  clips
  bond
  bondcpp
)

# -----------------------------
# Register node
# -----------------------------
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "cx::CLIPSEnvManager"
  EXECUTABLE cx_node
  EXECUTOR MultiThreadedExecutor
)

ament_package()
