cmake_minimum_required(VERSION 3.5)
project(arm_move)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(move_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(darknet_ros_msgs REQUIRED)

add_executable(arm_move src/arm_move.cpp)
ament_target_dependencies(
  arm_move
  rclcpp
  move_interfaces
  darknet_ros_msgs
)

add_executable(move_service src/move_service.cpp)
ament_target_dependencies(
  move_service
  rclcpp
  move_interfaces
  moveit_ros_planning_interface
)

install(TARGETS move_service arm_move
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
  
ament_package()
