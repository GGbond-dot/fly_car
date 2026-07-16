#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "activity_control_pkg/route_target_publisher.hpp"

namespace activity_control_pkg
{

// 覆盖生成器(飞车):把地面遍历航点灌入同进程的 RouteTargetPublisher(直接调 addTarget,
// 不过 DDS)。地面航点 z = cruise_z_cm;起飞/飞越由后续障碍决策 + 航点 z 编码处理。
//
// 两种走法(mode 参数):
//   - "l_path"(飞车任务默认):L 形,只转 1 个直角弯。飞车地面没调好、不想多走 ——
//     从固定起点走一段到障碍物前(拐角)→ 转 90° → 再走一段(终点)→ 起飞飞越。
//     用 l_start/l_corner/l_end 参数(map 系 cm,尺寸没量先占位)。不吃区域内容。
//   - "boustrophedon":弓字形,贴 1m 网格来回扫,用于单机自测或需整片覆盖时。
//     区域来自 /coverage_area(地面站框选→跨机 UDP→飞车 xmachine_bridge 本地发布)。
//
// 触发:收到 /coverage_area 即触发(l_path 模式下区域内容仅当"开跑"信号,几何用参数);
//   auto_start=true 时开机延时自触发(无地面站)。见 docs/coverage_flyover_mission_design.md。
class CoverageGeneratorNode : public rclcpp::Node
{
public:
  explicit CoverageGeneratorNode(
    std::shared_ptr<RouteTargetPublisherNode> route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onRegion(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void dispatch(double x_min_cm, double x_max_cm, double y_min_cm, double y_max_cm);
  // 弓字形:区域吸附网格后来回扫。
  void generateBoustrophedon(double x_min_cm, double x_max_cm, double y_min_cm, double y_max_cm);
  // L 形:起点→拐角(障碍前)→终点,1 个直角弯,用 l_* 参数(不吃区域)。
  void generateLPath();

  std::shared_ptr<RouteTargetPublisherNode> route_node_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr region_sub_;

  std::string mode_;      // "l_path"(默认) | "boustrophedon"

  // --- l_path 参数(map 系 cm,现场量了填)---
  double l_start_x_cm_, l_start_y_cm_;    // 飞车固定起点(仅用于算第一段朝向)
  double l_corner_x_cm_, l_corner_y_cm_;  // 拐角:走到障碍物前的点
  double l_end_x_cm_, l_end_y_cm_;        // 转弯后第二段终点(走完起飞)

  // --- boustrophedon 参数 ---
  double x_min_cm_, x_max_cm_, y_min_cm_, y_max_cm_;  // auto_start 时的场地
  double grid_cell_cm_;   // 网格格子边长(与 web GridHelper 一致,默认 100cm=1m)
  double lane_cells_;     // 行距 = grid_cell_cm * lane_cells(整数倍,默认 1)
  double lane_spacing_cm_;

  double cruise_z_cm_;
  double start_delay_s_;
  bool auto_start_;       // true=开机自触发;false=等 /coverage_area
  bool generated_ = false;  // 一次性:生成过就忽略后续("点一下就遍历"语义)
  std::vector<int64_t> land_after_indices_;  // 这些下标的航点打 land_after 标志(飞到后原地下降)
};

}  // namespace activity_control_pkg
