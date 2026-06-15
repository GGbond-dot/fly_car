#include "activity_control_pkg/coverage_generator.hpp"

#include <algorithm>
#include <chrono>

namespace activity_control_pkg
{

CoverageGeneratorNode::CoverageGeneratorNode(
  std::shared_ptr<RouteTargetPublisherNode> route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("coverage_generator", options),
  route_node_(std::move(route_node))
{
  // ⚠ 占位默认值,必须按实际场地标定(map 系,cm)
  x_min_cm_ = declare_parameter("area_x_min_cm", 0.0);
  x_max_cm_ = declare_parameter("area_x_max_cm", 500.0);
  y_min_cm_ = declare_parameter("area_y_min_cm", 0.0);
  y_max_cm_ = declare_parameter("area_y_max_cm", 300.0);
  lane_spacing_cm_ = declare_parameter("lane_spacing_cm", 50.0);
  cruise_z_cm_ = declare_parameter("cruise_z_cm", 4.0);     // 地面巡航高度
  start_delay_s_ = declare_parameter("start_delay_s", 2.0); // 等 TF/建图稳定再灌点
  // 这些下标的航点打 land_after 标志:飞到后原地垂直下降回地面(落点由你指定)
  land_after_indices_ = declare_parameter("land_after_indices", std::vector<int64_t>{});

  // 一次性定时器:延迟 start_delay_s 后灌点(给 Cartographer/TF 起来的时间)
  start_timer_ = create_wall_timer(
    std::chrono::duration<double>(start_delay_s_),
    [this]() {
      start_timer_->cancel();
      generateAndInject();
    });

  RCLCPP_INFO(get_logger(),
    "coverage_generator: 场地 x[%.0f,%.0f] y[%.0f,%.0f]cm 行距=%.0f 巡航z=%.0f 延迟=%.1fs",
    x_min_cm_, x_max_cm_, y_min_cm_, y_max_cm_, lane_spacing_cm_, cruise_z_cm_, start_delay_s_);
}

void CoverageGeneratorNode::generateAndInject()
{
  if (lane_spacing_cm_ <= 1.0 || x_max_cm_ <= x_min_cm_ || y_max_cm_ <= y_min_cm_) {
    RCLCPP_ERROR(get_logger(), "coverage_generator: 场地/行距参数非法,放弃生成");
    return;
  }

  // 弓字形:沿 x 方向来回扫,每扫完一行在 y 方向步进 lane_spacing。
  //   正向行 yaw=0(+x),反向行 yaw=180(-x);行首/行尾各一个航点。
  auto is_land_idx = [this](std::size_t idx) {
    return std::find(land_after_indices_.begin(), land_after_indices_.end(),
      static_cast<int64_t>(idx)) != land_after_indices_.end();
  };

  std::size_t count = 0;
  bool forward = true;
  for (double y = y_min_cm_; y <= y_max_cm_ + 1e-6; y += lane_spacing_cm_) {
    const double x_a = forward ? x_min_cm_ : x_max_cm_;
    const double x_b = forward ? x_max_cm_ : x_min_cm_;
    const double yaw = forward ? 0.0 : 180.0;
    route_node_->addTarget(Target{x_a, y, cruise_z_cm_, yaw, is_land_idx(count)});
    route_node_->addTarget(Target{x_b, y, cruise_z_cm_, yaw, is_land_idx(count + 1)});
    count += 2;
    forward = !forward;
  }

  RCLCPP_INFO(get_logger(),
    "coverage_generator: 已灌入 %zu 个弓字形地面航点", count);
}

}  // namespace activity_control_pkg
