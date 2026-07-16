#include "activity_control_pkg/coverage_generator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace activity_control_pkg
{

CoverageGeneratorNode::CoverageGeneratorNode(
  std::shared_ptr<RouteTargetPublisherNode> route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("coverage_generator", options),
  route_node_(std::move(route_node))
{
  mode_ = declare_parameter<std::string>("mode", "l_path");  // 飞车任务默认走 L 形

  // --- l_path 参数(map 系 cm,现场量了填;占位)---
  l_start_x_cm_ = declare_parameter("l_start_x_cm", 0.0);
  l_start_y_cm_ = declare_parameter("l_start_y_cm", 0.0);
  l_corner_x_cm_ = declare_parameter("l_corner_x_cm", 300.0);  // 走到障碍物前
  l_corner_y_cm_ = declare_parameter("l_corner_y_cm", 0.0);
  l_end_x_cm_ = declare_parameter("l_end_x_cm", 300.0);        // 转 90° 后第二段终点
  l_end_y_cm_ = declare_parameter("l_end_y_cm", 200.0);

  // --- boustrophedon 参数(auto_start 自测场地) ---
  x_min_cm_ = declare_parameter("area_x_min_cm", 0.0);
  x_max_cm_ = declare_parameter("area_x_max_cm", 500.0);
  y_min_cm_ = declare_parameter("area_y_min_cm", 0.0);
  y_max_cm_ = declare_parameter("area_y_max_cm", 300.0);
  grid_cell_cm_ = declare_parameter("grid_cell_cm", 100.0);  // 与 web GridHelper 一致:1m/格
  lane_cells_ = declare_parameter("lane_cells", 1.0);        // 行距=格子整数倍
  lane_spacing_cm_ = grid_cell_cm_ * std::max(1.0, lane_cells_);

  cruise_z_cm_ = declare_parameter("cruise_z_cm", 4.0);      // 地面巡航高度
  start_delay_s_ = declare_parameter("start_delay_s", 2.0);  // 等 TF/建图稳定再灌点
  auto_start_ = declare_parameter("auto_start", false);      // 默认等地面站信号
  // 这些下标的航点打 land_after 标志:飞到后原地垂直下降回地面(落点由你指定)
  land_after_indices_ = declare_parameter("land_after_indices", std::vector<int64_t>{});

  // 触发:地面站点击→跨机 UDP→飞车 xmachine_bridge 本地发布 /coverage_area。
  // latched(transient_local+reliable):即使信号先于本节点发布也能收到。
  region_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/coverage_area", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CoverageGeneratorNode::onRegion, this, std::placeholders::_1));

  if (auto_start_) {
    // 无地面站自测:延迟后自触发(给 Cartographer/TF 起来的时间)
    start_timer_ = create_wall_timer(
      std::chrono::duration<double>(start_delay_s_),
      [this]() {
        start_timer_->cancel();
        dispatch(x_min_cm_, x_max_cm_, y_min_cm_, y_max_cm_);
      });
  }

  RCLCPP_INFO(get_logger(),
    "coverage_generator: mode=%s,%s,巡航z=%.0f",
    mode_.c_str(),
    auto_start_ ? "auto_start 开机自触发" : "等 /coverage_area 信号",
    cruise_z_cm_);
}

void CoverageGeneratorNode::onRegion(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  // l_path 模式下区域内容仅当"开跑"信号(几何用参数);boustrophedon 用区域四元组。
  if (mode_ == "boustrophedon" && msg->data.size() < 4) {
    RCLCPP_WARN(get_logger(), "coverage_generator: /coverage_area 少于 4 个值,忽略");
    return;
  }
  const double x0 = msg->data.size() > 0 ? msg->data[0] : 0.0;
  const double x1 = msg->data.size() > 1 ? msg->data[1] : 0.0;
  const double y0 = msg->data.size() > 2 ? msg->data[2] : 0.0;
  const double y1 = msg->data.size() > 3 ? msg->data[3] : 0.0;
  dispatch(x0, x1, y0, y1);
}

void CoverageGeneratorNode::dispatch(
  double x_min_cm, double x_max_cm, double y_min_cm, double y_max_cm)
{
  if (generated_) {
    RCLCPP_INFO(get_logger(), "coverage_generator: 已生成过,忽略后续信号");
    return;
  }
  if (mode_ == "l_path") {
    generateLPath();  // 用 l_* 参数,忽略区域
  } else {
    generateBoustrophedon(x_min_cm, x_max_cm, y_min_cm, y_max_cm);
  }
}

void CoverageGeneratorNode::generateLPath()
{
  // L 形:飞车已在起点,第一段走到拐角(障碍前),转 90°,第二段走到终点。两段各一个航点,
  //   z=cruise_z 全程地面;yaw 沿各段行进方向。两点都不打 land_after —— 这只是"地面 L 段"。
  //   ⚠ 走完 L 后的"起飞→飞越障碍→覆盖另一边"属于飞越逻辑(航点 z 编码/障碍决策),不在本
  //   生成器职责内,由任务编排在 L 终点后追加高 z 航点触发,后续单独接。
  const double yaw1 = std::atan2(
    l_corner_y_cm_ - l_start_y_cm_, l_corner_x_cm_ - l_start_x_cm_) * 180.0 / M_PI;
  const double yaw2 = std::atan2(
    l_end_y_cm_ - l_corner_y_cm_, l_end_x_cm_ - l_corner_x_cm_) * 180.0 / M_PI;
  const double turn = std::fabs(std::remainder(yaw2 - yaw1, 360.0));
  if (std::fabs(turn - 90.0) > 20.0) {
    RCLCPP_WARN(get_logger(),
      "coverage_generator: L 形拐角=%.0f° 偏离 90°(拐角/终点参数没标定好?),仍按参数生成", turn);
  }
  route_node_->addTarget(Target{l_corner_x_cm_, l_corner_y_cm_, cruise_z_cm_, yaw1, false});
  route_node_->addTarget(Target{l_end_x_cm_, l_end_y_cm_, cruise_z_cm_, yaw2, false});
  generated_ = true;
  RCLCPP_INFO(get_logger(),
    "coverage_generator[L形]: 拐角(%.0f,%.0f)yaw=%.0f → 终点(%.0f,%.0f)yaw=%.0f,转%.0f°(地面段,起飞另接)",
    l_corner_x_cm_, l_corner_y_cm_, yaw1, l_end_x_cm_, l_end_y_cm_, yaw2, turn);
}

void CoverageGeneratorNode::generateBoustrophedon(
  double x_min_cm, double x_max_cm, double y_min_cm, double y_max_cm)
{
  const double g = grid_cell_cm_;
  if (g <= 1.0 || lane_spacing_cm_ <= 1.0) {
    RCLCPP_ERROR(get_logger(), "coverage_generator: 格子/行距非法,放弃");
    return;
  }
  // 吸附到网格线(向内取整:边界落在格线上,"路径按格子走")
  const double xa = std::ceil(x_min_cm / g) * g;
  const double xb = std::floor(x_max_cm / g) * g;
  const double ya = std::ceil(y_min_cm / g) * g;
  const double yb = std::floor(y_max_cm / g) * g;
  if (xb <= xa || yb <= ya) {
    RCLCPP_ERROR(get_logger(),
      "coverage_generator: 区域吸附到网格后为空 x[%.0f,%.0f] y[%.0f,%.0f],放弃", xa, xb, ya, yb);
    return;
  }

  // 弓字形:沿 x 来回扫,每行 y 步进 lane_spacing(=格子整数倍)。正向 yaw=0、反向 yaw=180。
  auto is_land_idx = [this](std::size_t idx) {
    return std::find(land_after_indices_.begin(), land_after_indices_.end(),
      static_cast<int64_t>(idx)) != land_after_indices_.end();
  };
  std::size_t count = 0;
  bool forward = true;
  for (double y = ya; y <= yb + 1e-6; y += lane_spacing_cm_) {
    const double x_a = forward ? xa : xb;
    const double x_b = forward ? xb : xa;
    const double yaw = forward ? 0.0 : 180.0;
    route_node_->addTarget(Target{x_a, y, cruise_z_cm_, yaw, is_land_idx(count)});
    route_node_->addTarget(Target{x_b, y, cruise_z_cm_, yaw, is_land_idx(count + 1)});
    count += 2;
    forward = !forward;
  }
  generated_ = true;
  RCLCPP_INFO(get_logger(),
    "coverage_generator[弓字形]: 区域 x[%.0f,%.0f] y[%.0f,%.0f]cm 吸附网格,灌入 %zu 个航点",
    xa, xb, ya, yb, count);
}

}  // namespace activity_control_pkg
