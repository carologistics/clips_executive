cmake_minimum_required(VERSION 3.16...3.31)
project(cx_z3_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  set(CX_Z3_VERSION z3-4.12.2)
  set(CX_Z3_PATCHES ${CMAKE_CURRENT_SOURCE_DIR}/4.12.2.patch)
else()
  set(CX_Z3_VERSION z3-4.16.0)
  set(CX_Z3_PATCHES "")
endif()

ament_vendor(z3
  VCS_URL https://github.com/Z3Prover/z3.git
  VCS_TYPE git
  VCS_VERSION ${CX_Z3_VERSION}
  PATCHES ${CX_Z3_PATCHES}
  CMAKE_ARGS
    -DZ3_BUILD_LIBZ3_SHARED=ON
    -DZ3_BUILD_LIBZ3_STATIC=OFF
    -DZ3_BUILD_PYTHON_BINDINGS=OFF
    -DZ3_BUILD_JAVA_BINDINGS=OFF
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_CXX_FLAGS="-w"
)

ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/opt/cx_z3_vendor/include)
ament_package()
