cmake_minimum_required(VERSION 3.5)
project(qrb_ros_manipulator_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ManipulatorException.msg"
  "srv/ManipulatorClawGetStatus.srv"
  "srv/ManipulatorClawControl.srv"
  "srv/ManipulatorGetControlMode.srv"
  "srv/ManipulatorGetJointPose.srv"
  "srv/ManipulatorGetTcpPose.srv"
  "srv/ManipulatorMoveJointPose.srv"
  "srv/ManipulatorMoveTcpPose.srv"
  "srv/ManipulatorSetControlMode.srv"
  "srv/ManipulatorTargetReachable.srv"
  DEPENDENCIES std_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

