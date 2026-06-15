#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "activity_control_pkg/route_target_publisher.hpp"

namespace activity_control_pkg
{

// 障碍决策(简化版,见用户思路):xy 航点不变,只控制“何时切飞行”。
// 订阅 /detected_obstacle 的 path_dist,当墙逼近到 approach_threshold(以车为心半径 0.6m)以内:
//   1. 插一个“当前 xy + z=flyover”的航点 → 原地垂直起飞(insertNext);
//   2. setFlightMode(true) → 后续原 xy 航点全部顶成 z=flyover,沿原路线在空中飞越。
// 落地暂不处理(触发后保持飞行,后续再定)。
//
// 任务里只有这一道要飞越的墙,默认 single_shot:触发一次后停发 /obstacle_detect_enable 并自锁。
class ObstacleDecisionNode : public rclcpp::Node
{
public:
  explicit ObstacleDecisionNode(
    std::shared_ptr<RouteTargetPublisherNode> route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void obstacleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  bool getCarPose(double & x_m, double & y_m, double & yaw_rad) const;
  void publishDetectEnable(bool on);

  std::shared_ptr<RouteTargetPublisherNode> route_node_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detect_enable_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string map_frame_;
  std::string laser_link_frame_;

  double approach_threshold_m_;  // path_dist ≤ 此值触发(以车为心半径)
  double flyover_z_cm_;          // 越障飞行高度
  bool single_shot_;

  bool done_ = false;
};

}  // namespace activity_control_pkg
