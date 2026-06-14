cmake_minimum_required(VERSION 3.16...3.31)
project(cx_bringup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cx_ros_comm_gen REQUIRED)
find_package(cx_protobuf_plugin REQUIRED)

set(dependencies
  ament_cmake
  ament_cmake_python
  cx_ros_comm_gen
  cx_protobuf_plugin
)

# generate bindings to interface with ROS messages, such as std_msgs::msg::String
cx_generate_msg_bindings("std_msgs" "String")

# generate bindings to interface with ROS services, such as std_srvs::srv::SetBool
cx_generate_srv_bindings("std_srvs" "SetBool")

# generate bindings to interface with ROS services, such as example_interfaces::action::Fibonacci
cx_generate_action_bindings("example_interfaces" "Fibonacci")

# generate protobuf plugin with linked proto files
cx_generate_linked_protobuf_plugin_from_proto("BringupProtobufPlugin" proto/cx_bringup/SearchRequest.proto)

install(DIRECTORY launch params clips proto DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/test_service.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_ament_index_plugin
    test/test_ament_index_plugin.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_config_plugin
    test/test_config_plugin.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_executive_plugin
    test/test_executive_plugin.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_fibonacci_action_plugin
    test/test_fibonacci_action_plugin.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_file_load_plugin
    test/test_file_load_plugin.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_protobuf_plugin
    test/test_protobuf_plugin.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_protobuf_linked_plugin
    test/test_protobuf_linked_plugin.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_ros_msgs_plugin
    test/test_ros_msgs_plugin.py
    TIMEOUT 60
  )
  # Tests only for Jazzy and newer
  if("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "jazzy")
    ament_add_pytest_test(test_ros_msgs_client_plugin
      test/test_ros_msgs_client_plugin.py
      TIMEOUT 60
    )
  endif()

  if("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "kilted")
    ament_add_pytest_test(test_ros_msgs_service_plugin
      test/test_ros_msgs_service_plugin.py
      TIMEOUT 60
    )
    ament_add_pytest_test(test_ros_msgs_action_client_plugin
      test/test_ros_msgs_action_client_plugin.py
      TIMEOUT 60
    )
  endif()

  ament_add_pytest_test(test_ros_param_plugin
    test/test_ros_param_plugin.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_set_bool_srv_plugin
    test/test_set_bool_srv_plugin.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_string_msg_plugin
    test/test_string_msg_plugin.py
    TIMEOUT 60
  )
endif()

ament_python_install_package(cx_bringup_test_utils
  PACKAGE_DIR test/utils
)

ament_export_dependencies(${dependencies})

ament_package()
