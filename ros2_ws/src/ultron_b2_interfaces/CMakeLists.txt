cmake_minimum_required(VERSION 3.8)
project(ultron_b2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalizationState.msg"
  "msg/MotionGateStatus.msg"
  "msg/OperatorEnable.msg"
  "msg/LadderDetection.msg"
  "msg/LadderDetectionArray.msg"
  "msg/LadderCandidate.msg"
  "msg/LadderCandidateArray.msg"
  "msg/MissionStatus.msg"
  "msg/MissionEvent.msg"
  "msg/CoverageReport.msg"
  "msg/MapInfo.msg"
  "srv/Arm.srv"
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  "srv/SetPatrolZone.srv"
  "srv/SimScenario.srv"
  "action/NavigateToView.action"
  "action/CaptureViews.action"
  "action/RunMission.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs action_msgs
)

# JSON <-> msg converters live in a separate Python package so they do not collide
# with the rosidl-generated `ultron_b2_interfaces` Python package.
ament_python_install_package(ultron_b2_contracts)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_json_roundtrip test/test_json_roundtrip.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py:${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
