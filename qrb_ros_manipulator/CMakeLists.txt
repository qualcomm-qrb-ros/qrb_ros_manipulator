cmake_minimum_required(VERSION 3.8)
project(qrb_ros_manipulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-reorder)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(qrb_manipulator_controller REQUIRED)
find_package(qrb_ros_manipulator_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories("${PROJECT_SOURCE_DIR}/include")

add_executable(qrb_ros_manipulator
  src/main.cpp
  src/manipulator_controller.cpp
  src/topic/manipulator_exception_publisher.cpp
  src/service/manipulator_control_service_server.cpp
  )

ament_target_dependencies(qrb_ros_manipulator
  qrb_manipulator_controller
  qrb_ros_manipulator_msgs
  rclcpp
  std_msgs
  geometry_msgs
  )
#target_link_libraries(qrb_ros_manipulator qrb_manipulator_controller)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
qrb_ros_manipulator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
