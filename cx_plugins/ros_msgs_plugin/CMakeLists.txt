cmake_minimum_required(VERSION 3.16...3.31)
project(cx_ros_msgs_plugin)

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
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cx_plugin REQUIRED)
find_package(cx_utils REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(ICU REQUIRED COMPONENTS uc i18n)
find_package(action_msgs REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)


# Extract ROS distro from ament_cmake version or environment
if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
else()
  message(FATAL_ERROR "ROS_DISTRO environment variable not set")
endif()

if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
else()
  message(FATAL_ERROR "ROS_DISTRO environment variable not set")
endif()

# HAVE_GENERIC_CLIENT: jazzy and above
if(ROS_DISTRO STRGREATER_EQUAL "jazzy")
  set(HAVE_GENERIC_CLIENT TRUE)
endif()

# HAVE_GENERIC_SERVICE: kilted and above
if(ROS_DISTRO STRGREATER_EQUAL "kilted")
  set(HAVE_GENERIC_SERVICE TRUE)
endif()

# HAVE_GENERIC_ACTION_CLIENT: kilted and above
if(ROS_DISTRO STRGREATER_EQUAL "kilted")
  set(HAVE_GENERIC_ACTION_CLIENT TRUE)
endif()

add_library(${PROJECT_NAME} SHARED
  src/ros_msgs_plugin.cpp
  src/ros_msgs_client_impl.cpp
  src/ros_msgs_service_impl.cpp
  src/ros_msgs_action_client_impl.cpp
)

pluginlib_export_plugin_description_file(cx_plugin ros_msgs_plugin.xml)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ClipsNS::libclips_ns
  ICU::uc
  ICU::i18n
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  pluginlib::pluginlib
  cx_plugin::cx_plugin
  cx_utils::cx_utils
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
  ${action_msgs_TARGETS}
  rcutils::rcutils
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD ${CMAKE_CXX_STANDARD})

if(HAVE_GENERIC_ACTION_CLIENT)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_GENERIC_ACTION_CLIENT)
else()
  message(WARNING "Skipping support for action clients, missing support in rclcpp_action")
endif()

if(HAVE_GENERIC_SERVICE)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_GENERIC_SERVICE)
else()
  message(WARNING "Skipping support for service providers, missing support in rclcpp")
endif()

if(HAVE_GENERIC_CLIENT)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_GENERIC_CLIENT)
else()
  message(WARNING "Skipping support for service clients, missing support in rclcpp")
endif()

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES ros_msgs_plugin.xml
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
  rclcpp
  rclcpp_action
  pluginlib
  cx_plugin
  cx_utils
  clips_vendor
  clips
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  action_msgs
  rcutils
  ICU
)

ament_package()
