cmake_minimum_required(VERSION 3.8)
project(obstacle_detector_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(obstacle_detector
  src/obstacle_detector_node.cpp
  src/obstacle_detector_main.cpp
)
target_include_directories(obstacle_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(obstacle_detector
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
if(MSVC)
  target_compile_options(obstacle_detector PRIVATE /W4)
else()
  target_compile_options(obstacle_detector PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS
  obstacle_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
