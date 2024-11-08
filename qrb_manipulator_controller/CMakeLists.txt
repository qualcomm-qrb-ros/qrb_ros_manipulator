cmake_minimum_required(VERSION 3.5)
project(qrb_manipulator_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++11)
endif()

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)

#SET(INC_DIR /usr/include/lebai)
#SET(LINK_DIR /usr/lib/liblebai-cpp.so)
#SET(INC_DIR /usr/local/include/lebai)
#SET(LINK_DIR /usr/local/lib/liblebai-cpp.so)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(${PROJECT_NAME} SHARED
  src/lebai_implement.cpp
  src/manipulator_interface.cpp
  src/manipulator_management.cpp
  src/manipulator_sm.cpp
)

ament_target_dependencies( ${PROJECT_NAME}
  geometry_msgs
  )

option(ENABLE_TEST "Enable test specific functions" OFF)
if(ENABLE_TEST)
  add_definitions(-DENABLE_TEST)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/qrb_manipulator_controller
  ${INC_DIR}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
target_link_libraries(qrb_manipulator_controller lebai-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
