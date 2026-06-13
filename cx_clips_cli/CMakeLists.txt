cmake_minimum_required(VERSION 3.16...3.31)
project(cx_clips_cli)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cli
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_clips_cli          # test target name (ctest label)
    test/test_clips_cli.py  # path to your test file
    TIMEOUT 60
  )
endif()

#install(DIRECTORY
#  launch
#  params
#  DESTINATION share/${PROJECT_NAME}
#)

ament_package()
