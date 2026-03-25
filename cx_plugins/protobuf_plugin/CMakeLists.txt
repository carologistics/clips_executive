cmake_minimum_required(VERSION 3.16...3.31)
project(cx_protobuf_plugin)

# Uses C++20 or above
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cx_plugin REQUIRED)
find_package(cx_utils REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ProtobufComm REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)

# install generator script
install(
  FILES generator.py
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------
# clips_protobuf_comm library
# ------------------------------
add_library(clips_protobuf_comm SHARED src/communicator.cpp)

target_include_directories(clips_protobuf_comm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(clips_protobuf_comm PUBLIC
  protobuf::libprotobuf
  ProtobufComm::protobuf_comm
  ClipsNS::libclips_ns
  rclcpp_lifecycle::rclcpp_lifecycle
)

target_link_libraries(clips_protobuf_comm PUBLIC
  cx_utils::cx_utils
)

set_property(TARGET clips_protobuf_comm PROPERTY CXX_STANDARD ${CMAKE_CXX_STANDARD})

# ------------------------------
# main plugin library
# ------------------------------
add_library(${PROJECT_NAME} SHARED src/protobuf_plugin.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(cx_plugin protobuf_plugin.xml)

target_link_libraries(${PROJECT_NAME} PUBLIC
  clips_protobuf_comm
  ClipsNS::libclips_ns
  rclcpp::rclcpp
  pluginlib::pluginlib
  cx_plugin::cx_plugin
  cx_utils::cx_utils
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD ${CMAKE_CXX_STANDARD})

# ------------------------------
# Install rules
# ------------------------------
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY templates
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES protobuf_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY clips/
  DESTINATION share/${PROJECT_NAME}/clips/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  clips_protobuf_comm
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES cmake/cx_protobuf_sub_plugin.cmake
        DESTINATION share/${PROJECT_NAME}/cmake)

# ------------------------------
# Exports
# ------------------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  pluginlib
  cx_plugin
  cx_utils
  Protobuf
  ProtobufComm
  clips_vendor
  clips
)

ament_package(
  CONFIG_EXTRAS cmake/cx_protobuf_sub_plugin.cmake
)
