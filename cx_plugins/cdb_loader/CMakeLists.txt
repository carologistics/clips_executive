cmake_minimum_required(VERSION 3.8)
project(cx_cdb_loader_plugin)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cx_plugin REQUIRED)
find_package(cx_utils REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBPQXX REQUIRED IMPORTED_TARGET libpqxx)

set(dependencies
  ament_cmake
  rclcpp
  pluginlib
  cx_plugin
  cx_utils
  clips_vendor
  clips
)

include_directories(include)

set(CDB_SQL_INPUT ${CMAKE_CURRENT_SOURCE_DIR}/sql/schema.sql)
set(CDB_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(CDB_SQL_HEADER ${CDB_GENERATED_DIR}/cx_cdb_loader_plugin/schema_sql.hpp)

add_custom_command(
  OUTPUT ${CDB_SQL_HEADER}
  COMMAND ${CMAKE_COMMAND}
    -DINPUT=${CDB_SQL_INPUT}
    -DOUTPUT=${CDB_SQL_HEADER}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_sql.cmake
  DEPENDS
    ${CDB_SQL_INPUT}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_sql.cmake
  COMMENT "Embedding SQL schema into header file"
  VERBATIM
)

add_custom_target(
  cx_cdb_loader_schema_header
  DEPENDS ${CDB_SQL_HEADER}
  COMMENT "Ensuring embedded SQL schema is regenerated on change"
)

pluginlib_export_plugin_description_file(cx_plugin cdb_loader_plugin.xml)

add_library(${PROJECT_NAME} SHARED
  src/cdb_loader_plugin.cpp
  ${CDB_SQL_HEADER}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${CDB_GENERATED_DIR}
)

target_link_libraries(${PROJECT_NAME} ClipsNS::libclips_ns PkgConfig::LIBPQXX)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD ${CMAKE_CXX_STANDARD})

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES cdb_loader_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
