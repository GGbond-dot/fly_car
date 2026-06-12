#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace obstacle_detector_pkg
{

// map 系下的一个二维点
struct Pt2
{
  double x;
  double y;
};

// 折线拟合结果（顶点均为 map 系，单位米，按首尾相连有序）
struct Polyline
{
  bool             valid = false;
  std::vector<Pt2> vertices;     // 顶点串，相邻顶点连成一段，N 个顶点 = N-1 段
  double           total_length = 0.0;  // 折线总长
  double           path_dist = 0.0;     // 沿当前路线到阻挡墙的距离
};

// 激光雷达长障碍物（折线墙）检测：
//   借鉴 pillar_detector，靠 TF(map <- laser_link) 把每帧激光点变换到 map 系，
//   在 ROI 内取点，按邻近连成点链，用 Split-and-Merge(迭代端点法) 拟合成折线，
//   输出有序顶点串。由 /obstacle_detect_enable 控制启停；使能期间每帧拟合并连续发布。
class ObstacleDetectorNode : public rclcpp::Node
{
public:
  explicit ObstacleDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void targetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  // 单帧：TF 变换到 map 系 + ROI 过滤，返回按扫描序排列的 map 系点云
  std::vector<Pt2> collectRoiPoints(const sensor_msgs::msg::LaserScan & scan, Pt2 & car_xy_out);
  // 把有序点按邻近断点切成若干点链
  std::vector<std::vector<Pt2>> splitChains(const std::vector<Pt2> & pts) const;
  // Split-and-Merge 折线拟合（含近共线段合并）
  Polyline fitPolyline(const std::vector<Pt2> & chain) const;
  bool pathDistanceToPolyline(
    const Pt2 & car_xy, const Pt2 & target_xy, const Polyline & poly, double & path_dist) const;
  void publishObstacle(const Polyline & poly);

  // ── 参数 ──────────────────────────────────────────────────
  std::string scan_topic_;
  std::string enable_topic_;
  std::string map_frame_;
  std::string laser_link_frame_;

  // 障碍物期望出现的世界 ROI（map 系，单位米）。墙在车前方，按场地标定。
  double roi_x_min_m_;
  double roi_x_max_m_;
  double roi_y_min_m_;
  double roi_y_max_m_;

  // 折线拟合
  double chain_break_dist_m_;   // 相邻点距离超过此值则断链
  int    min_chain_points_;     // 点链点数少于此值不认为是墙
  double split_threshold_m_;    // Split-and-Merge：点到段距离超过此值则在该点劈开
  double merge_collinear_deg_;  // 相邻两段转角小于此值则合并（去碎段）
  double min_total_length_m_;   // 折线总长短于此值不认为是墙
  double path_corridor_half_width_m_;  // 当前路线两侧允许的阻挡检测宽度

  double tf_timeout_sec_;

  // 调参旁路：把落进 ROI 的 map 系原始点额外发一份，供录包离线重调。
  bool        publish_debug_points_;
  std::string debug_points_topic_;

  // ── 状态 ──────────────────────────────────────────────────
  bool enabled_;
  bool has_ground_target_ = false;
  Pt2 ground_target_{0.0, 0.0};
  mutable std::mutex mutex_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_points_pub_;
};

}  // namespace obstacle_detector_pkg
