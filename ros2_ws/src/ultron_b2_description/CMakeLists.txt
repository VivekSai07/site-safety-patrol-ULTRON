cmake_minimum_required(VERSION 3.8)
project(ultron_b2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf calibration config launch license
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py TIMEOUT 120)
endif()

ament_package()
