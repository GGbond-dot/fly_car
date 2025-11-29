cmake_minimum_required(VERSION 3.8)
project(activity_control_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

add_library(route_target_publisher SHARED
  src/route_target_publisher.cpp
)
target_include_directories(route_target_publisher PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
   $<INSTALL_INTERFACE:include>)
ament_target_dependencies(route_target_publisher
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)
if(MSVC)
  target_compile_options(route_target_publisher PRIVATE /W4)
else()
  target_compile_options(route_target_publisher PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(route_target_publisher_node src/route_target_publisher_main.cpp)
target_link_libraries(route_target_publisher_node route_target_publisher)
ament_target_dependencies(route_target_publisher_node rclcpp)

add_executable(route_test_node src/route_test_node.cpp)
target_link_libraries(route_test_node route_target_publisher)
ament_target_dependencies(route_test_node rclcpp)

install(TARGETS route_target_publisher
  EXPORT export_route_target_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  route_target_publisher_node
  route_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py")

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)
ament_export_include_directories(include)
ament_export_targets(export_route_target_publisher HAS_LIBRARY_TARGET)

ament_package()
