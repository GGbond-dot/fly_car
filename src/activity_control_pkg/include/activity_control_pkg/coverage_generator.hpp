#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "activity_control_pkg/route_target_publisher.hpp"

namespace activity_control_pkg
{

// 覆盖生成器:把已知场地按行距生成弓字形(boustrophedon)地面航点,
// 启动后一次性灌入同进程的 RouteTargetPublisher(直接调 addTarget,不过 DDS)。
// 全部航点 z = cruise_z_cm(地面巡航),由后续障碍决策在遇墙处插入起降航点。
//
// 场地参数为占位默认值,必须按实际场地标定(见 docs/coverage_flyover_mission_design.md §六)。
class CoverageGeneratorNode : public rclcpp::Node
{
public:
  explicit CoverageGeneratorNode(
    std::shared_ptr<RouteTargetPublisherNode> route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void generateAndInject();

  std::shared_ptr<RouteTargetPublisherNode> route_node_;
  rclcpp::TimerBase::SharedPtr start_timer_;

  double x_min_cm_;
  double x_max_cm_;
  double y_min_cm_;
  double y_max_cm_;
  double lane_spacing_cm_;
  double cruise_z_cm_;
  double start_delay_s_;
  std::vector<int64_t> land_after_indices_;  // 这些下标的航点打 land_after 标志(飞到后原地下降)
};

}  // namespace activity_control_pkg
