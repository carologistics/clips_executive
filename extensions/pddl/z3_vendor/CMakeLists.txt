cmake_minimum_required(VERSION 3.8)
project(z3_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

set(Z3_DIR ${CMAKE_CURRENT_SOURCE_DIR}/z3-4.12.0-x64-glibc-2.35)

add_library(z3 INTERFACE)
target_link_libraries(
  z3
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/z3-4.12.0-x64-glibc-2.35/bin/libz3.so>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libz3.so>)
target_include_directories(z3 INTERFACE $<BUILD_INTERFACE:${Z3_DIR}/include>
                                        $<INSTALL_INTERFACE:include>)
# Install the precompiled binary
install(DIRECTORY z3-4.12.0-x64-glibc-2.35/bin/ DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY z3-4.12.0-x64-glibc-2.35/include/ DESTINATION include/)

install(
  TARGETS z3
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
