cmake_minimum_required(VERSION 3.16...3.31)
project(cx_tf2_pose_tracker_plugin)

# Uses C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cx_plugin REQUIRED)
find_package(cx_utils REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/tf2_pose_tracker_plugin.cpp)

pluginlib_export_plugin_description_file(cx_plugin tf2_pose_tracker_plugin.xml)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ClipsNS::libclips_ns
  pluginlib::pluginlib
  cx_plugin::cx_plugin
  cx_utils::cx_utils
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD ${CMAKE_CXX_STANDARD})

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES tf2_pose_tracker_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  pluginlib
  cx_plugin
  cx_utils
  clips_vendor
  clips
  tf2_ros
  geometry_msgs
)

ament_package()
