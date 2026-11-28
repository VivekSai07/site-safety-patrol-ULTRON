cmake_minimum_required(VERSION 3.8)
project(ultron_b2_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(tf_authority_check src/tf_authority_check.cpp)
target_compile_features(tf_authority_check PUBLIC cxx_std_17)
ament_target_dependencies(tf_authority_check rclcpp tf2_msgs diagnostic_msgs)
install(TARGETS tf_authority_check DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/preflight scripts/placeholder DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_env PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_preflight test/test_preflight.py APPEND_ENV ${_env} TIMEOUT 60)
  ament_add_pytest_test(test_launch_structure test/test_launch_structure.py APPEND_ENV ${_env} TIMEOUT 60)
  ament_add_pytest_test(test_tf_authority test/test_tf_authority.py APPEND_ENV ${_env} TIMEOUT 180)
  ament_add_pytest_test(test_sim_launch test/test_sim_launch.py APPEND_ENV ${_env} TIMEOUT 180)
  ament_add_pytest_test(test_mission_launch test/test_mission_launch.py APPEND_ENV ${_env} TIMEOUT 180)
endif()

ament_package()
