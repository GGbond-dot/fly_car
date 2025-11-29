#include <clocale>

#include "activity_control_pkg/route_target_publisher.hpp"

int main(int argc, char ** argv)
{
  std::setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<activity_control_pkg::RouteTargetPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
