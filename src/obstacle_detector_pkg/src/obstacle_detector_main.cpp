#include "obstacle_detector_pkg/obstacle_detector_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<obstacle_detector_pkg::ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
