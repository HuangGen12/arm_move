cmake_minimum_required(VERSION 3.5)
project(bottle_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(darknet_ros_msgs  REQUIRED)
find_package(darknet_ros  REQUIRED)
find_package(cv_bridge  REQUIRED)
find_package(realsense2_camera  REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(realsense2)
find_package(realsense2_camera_msgs REQUIRED)

include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ${realsense_INCLUDE_DIR}
)

if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()


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


add_executable(rs_node_listener_node src/rs2_node_listener.cpp)
#add_executable(pc_handler_node src/pc_handler.cpp)

target_include_directories(rs_node_listener_node
  PRIVATE ${realsense2_INCLUDE_DIR})

target_link_libraries(rs_node_listener_node
    ${realsense2_LIBRARY}
)

ament_target_dependencies(rs_node_listener_node rclcpp sensor_msgs geometry_msgs darknet_ros_msgs tf2 cv_bridge realsense2 realsense2_camera
tf2_kdl
tf2_geometry_msgs
tf2_eigen
realsense2_camera_msgs) 
#ament_target_dependencies(pc_handler_node)

install(TARGETS
  rs_node_listener_node
  #pc_handler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()