#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "uart_to_stm32/uart_to_stm32.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("uart_to_stm32_node");

  node->declare_parameter<double>("update_rate", 100.0);
  node->declare_parameter<std::string>("source_frame", "map");
  node->declare_parameter<std::string>("target_frame", "laser_link");

  const auto update_rate = node->get_parameter("update_rate").as_double();
  const auto source_frame = node->get_parameter("source_frame").as_string();
  const auto target_frame = node->get_parameter("target_frame").as_string();

  try {
    auto app = std::make_shared<uart_to_stm32::UartToStm32>(node);
    if (!app->initialize(update_rate, source_frame, target_frame)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize UartToStm32");
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }

    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Exception in main: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
