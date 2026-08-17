cmake_minimum_required(VERSION 3.8)
project(laser_array_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(laser_array_driver src/laser_array_driver.cpp)
ament_target_dependencies(laser_array_driver rclcpp std_msgs)

target_link_libraries(laser_array_driver
  Boost::thread
  Boost::system
)

add_executable(laser_array_ground_node src/laser_array_ground_node.cpp)
ament_target_dependencies(laser_array_ground_node rclcpp std_msgs)

target_link_libraries(laser_array_ground_node
  Boost::thread
  Boost::system
)

install(TARGETS laser_array_driver laser_array_ground_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)


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

ament_package()
