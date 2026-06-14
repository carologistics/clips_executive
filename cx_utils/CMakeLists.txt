cmake_minimum_required(VERSION 3.16...3.31)
project(cx_utils)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if($ENV{ROS_DISTRO} MATCHES "humble")
  add_compile_definitions(HUMBLE)
elseif($ENV{ROS_DISTRO} MATCHES "jazzy")
  add_compile_definitions(JAZZY)
elseif($ENV{ROS_DISTRO} MATCHES "kilted")
  add_compile_definitions(KILTED)
elseif($ENV{ROS_DISTRO} MATCHES "rolling")
  add_compile_definitions(ROLLING)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog_vendor REQUIRED)
find_package(spdlog REQUIRED)
find_package(clips_vendor REQUIRED)
find_package(clips REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(fmt)

set(CX_UTILS_SOURCES
  src/clips_env_context.cpp
  src/param_utils.cpp
)

# -----------------------------
# Library target
# -----------------------------
add_library(${PROJECT_NAME} SHARED ${CX_UTILS_SOURCES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  spdlog::spdlog
  ClipsNS::libclips_ns
  ament_index_cpp::ament_index_cpp
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD ${CMAKE_CXX_STANDARD})

# -----------------------------
# std::format / fmt detection
# -----------------------------
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_FLAGS "-std=c++20")

check_cxx_source_compiles("
  #include <format>
  #include <string>
  int main() {
      auto s = std::format(\"Hello {}\", 42);
      return s.size();
  }
" HAVE_STD_FORMAT)

if(HAVE_STD_FORMAT)
  message(STATUS "std::format is supported")
else()
  message(STATUS "std::format is NOT supported, checking for bundled fmt")
  include(CheckIncludeFileCXX)
  check_include_file_cxx("spdlog/fmt/fmt.h" SPDLOG_HAS_BUNDLED_FMT CMAKE_REQUIRED_LIBRARIES spdlog)
  if(SPDLOG_HAS_BUNDLED_FMT)
    message(STATUS "spdlog has bundled fmt, using that")
  else()
    message(STATUS "spdlog has not bundled fmt, using external fmt")
    find_package(fmt REQUIRED)
    target_compile_definitions(${PROJECT_NAME} PUBLIC SPDLOG_FMT_EXTERNAL=1)
    ament_export_definitions(SPDLOG_FMT_EXTERNAL)
    target_link_libraries(${PROJECT_NAME} PUBLIC fmt::fmt)
  endif()
endif()

get_target_property(AMENT_INDEX_CPP_INCLUDES ament_index_cpp::ament_index_cpp INTERFACE_INCLUDE_DIRECTORIES)
set(CMAKE_REQUIRED_INCLUDES ${AMENT_INDEX_CPP_INCLUDES})
include(CheckIncludeFileCXX)
check_include_file_cxx("ament_index_cpp/get_package_share_path.hpp" HAVE_GET_PACKAGE_SHARE_PATH REQUIRED_LIBARIES ament_index_cpp::ament_index_cpp)

if(HAVE_GET_PACKAGE_SHARE_PATH)
  message(STATUS "get_package_share_path is supported")
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_GET_PACKAGE_SHARE_PATH=1)
else()
  message(STATUS "get_package_share_path is not supported, using get_package_share_directory instead")
endif()

# -----------------------------
# Install rules
# -----------------------------
install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_resolve_files_test test/test_resolve_files.cpp)
  target_include_directories(${PROJECT_NAME}_resolve_files_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(${PROJECT_NAME}_resolve_files_test ${PROJECT_NAME})
  ament_add_gtest(${PROJECT_NAME}_param_utils_test test/test_param_utils.cpp)
  target_include_directories(${PROJECT_NAME}_param_utils_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_param_utils_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_clips_logger_test test/test_clips_logger.cpp)
  target_include_directories(${PROJECT_NAME}_clips_logger_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_clips_logger_test ${PROJECT_NAME})
endif()

# -----------------------------
# Export
# -----------------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  spdlog
  ament_index_cpp
)

ament_package()
