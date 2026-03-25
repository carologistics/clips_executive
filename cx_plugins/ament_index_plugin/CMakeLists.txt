cmake_minimum_required(VERSION 3.16...3.31)
project(cx_ament_index_plugin)

# use C++20 for std::format
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cx_plugin REQUIRED)
find_package(cx_utils REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)

add_library(${PROJECT_NAME} SHARED src/ament_index_plugin.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  cx_plugin::cx_plugin
  cx_utils::cx_utils
  ClipsNS::libclips_ns
)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD ${CMAKE_CXX_STANDARD})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(cx_plugin ament_index_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES ament_index_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  pluginlib
  cx_plugin
  cx_utils
  clips_vendor
  clips
)
ament_package()
