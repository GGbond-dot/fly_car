cmake_minimum_required(VERSION 3.8)
project(ground_chassis_pkg)

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

# 地面差速跟踪控制器(/target_position + /ground_enable -> /cmd_vel)
add_executable(diff_drive_controller src/diff_drive_controller.cpp)
ament_target_dependencies(diff_drive_controller
  rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# 地空互斥仲裁(按目标 z 发布 /ground_enable 与 /flight_enable)
add_executable(chassis_mux src/chassis_mux.cpp)
ament_target_dependencies(chassis_mux rclcpp std_msgs)

if(NOT MSVC)
  target_compile_options(diff_drive_controller PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(chassis_mux PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS
  diff_drive_controller
  chassis_mux
  DESTINATION lib/${PROJECT_NAME}
)

# Python 串口桥(SR5E1E3 $VW) + 轮速 bias 开环测量探针
install(PROGRAMS
  scripts/chassis_bridge.py
  scripts/bias_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py")

ament_package()
