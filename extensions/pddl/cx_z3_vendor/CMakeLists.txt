cmake_minimum_required(VERSION 3.16)
project(cx_z3_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(z3
  VCS_URL https://github.com/Z3Prover/z3.git
  VCS_TYPE git
  VCS_VERSION z3-4.12.2  # use a tag or commit
  PATCHES 4.12.2.patch
  CMAKE_ARGS
    -DZ3_BUILD_LIBZ3_SHARED=ON
    -DZ3_BUILD_LIBZ3_STATIC=OFF
    -DZ3_BUILD_PYTHON_BINDINGS=OFF
    -DZ3_BUILD_JAVA_BINDINGS=OFF
    -DCMAKE_BUILD_TYPE=Release
)

ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/opt/cx_z3_vendor/include)
ament_package()
