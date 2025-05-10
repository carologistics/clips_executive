cmake_minimum_required(VERSION 3.16...3.28)
project(cx_pddl_parser_plugin)

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
find_package(pluginlib REQUIRED)
find_package(cx_plugin REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)
find_package(cx_utils REQUIRED)
find_package(PddlParser REQUIRED)

set(dependencies
  ament_cmake
  rclcpp
  cx_plugin
  cx_utils
  pluginlib
  clips_vendor
  clips
)

# clips_pddl_parser helper lib

add_library(clips_pddl_parser SHARED src/clips_pddl_parser.cpp src/precondition_visitor.cpp src/effect_visitor.cpp)
target_link_libraries(
  clips_pddl_parser PddlParser::pddl_parser
  ClipsNS::libclips_ns spdlog::spdlog)
target_include_directories(
  clips_pddl_parser PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/> $<INSTALL_INTERFACE:include/>)

add_library(${PROJECT_NAME} SHARED src/pddl_parser_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} clips_pddl_parser
                                      ClipsNS::libclips_ns)

include_directories(include)

pluginlib_export_plugin_description_file(cx_plugin pddl_parser_plugin.xml)

# END OF PLUGINS REGISTRATION
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY clips/
  DESTINATION share/${PROJECT_NAME}/clips/${PROJECT_NAME}
)

install(
  FILES pddl_parser_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME} clips_pddl_parser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} clips_pddl_parser)
ament_export_dependencies(${dependencies} spdlog pddl_parser)

ament_package()
