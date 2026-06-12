#include "obstacle_detector_pkg/obstacle_detector_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace obstacle_detector_pkg
{
namespace
{

// 点 p 到无限直线 a-b 的垂直距离
double pointToLine(const Pt2 & p, const Pt2 & a, const Pt2 & b)
{
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-9) { return std::hypot(p.x - a.x, p.y - a.y); }
  return std::fabs((p.x - a.x) * dy - (p.y - a.y) * dx) / len;
}

// 点 p 到线段 a-b 的距离（带端点夹紧）
double pointToSegment(const Pt2 & p, const Pt2 & a, const Pt2 & b)
{
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double l2 = dx * dx + dy * dy;
  if (l2 < 1e-12) { return std::hypot(p.x - a.x, p.y - a.y); }
  double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2;
  t = std::max(0.0, std::min(1.0, t));
  const double px = a.x + t * dx, py = a.y + t * dy;
  return std::hypot(p.x - px, p.y - py);
}

// 求墙线段到当前路线段的最小距离，并返回对应路线进度 t（0=车，1=航点）。
bool segmentPathDistance(
  const Pt2 & route_start, const Pt2 & route_end, const Pt2 & wall_start, const Pt2 & wall_end,
  double & min_dist, double & route_t)
{
  const double rx = route_end.x - route_start.x;
  const double ry = route_end.y - route_start.y;
  const double sx = wall_end.x - wall_start.x;
  const double sy = wall_end.y - wall_start.y;
  const double route_l2 = rx * rx + ry * ry;
  if (route_l2 < 1e-12) { return false; }

  min_dist = std::numeric_limits<double>::max();
  route_t = 0.0;
  auto consider = [&](double t, double distance) {
      t = std::max(0.0, std::min(1.0, t));
      if (distance < min_dist) {
        min_dist = distance;
        route_t = t;
      }
    };

  const double cross = rx * sy - ry * sx;
  if (std::fabs(cross) > 1e-9) {
    const double qpx = wall_start.x - route_start.x;
    const double qpy = wall_start.y - route_start.y;
    const double t = (qpx * sy - qpy * sx) / cross;
    const double u = (qpx * ry - qpy * rx) / cross;
    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
      min_dist = 0.0;
      route_t = t;
      return true;
    }
  }

  for (const Pt2 & wall_point : {wall_start, wall_end}) {
    const double t = ((wall_point.x - route_start.x) * rx +
      (wall_point.y - route_start.y) * ry) / route_l2;
    const double clamped_t = std::max(0.0, std::min(1.0, t));
    const Pt2 route_point{route_start.x + clamped_t * rx, route_start.y + clamped_t * ry};
    consider(clamped_t, std::hypot(wall_point.x - route_point.x, wall_point.y - route_point.y));
  }
  consider(0.0, pointToSegment(route_start, wall_start, wall_end));
  consider(1.0, pointToSegment(route_end, wall_start, wall_end));
  return true;
}

// Douglas–Peucker：递归在残差最大处劈开，标记保留的顶点
void dpSplit(const std::vector<Pt2> & pts, int s, int e, double thr, std::vector<char> & keep)
{
  if (e <= s + 1) { return; }
  double max_d = -1.0;
  int max_k = -1;
  for (int k = s + 1; k < e; ++k) {
    const double d = pointToLine(pts[k], pts[s], pts[e]);
    if (d > max_d) { max_d = d; max_k = k; }
  }
  if (max_d > thr && max_k > 0) {
    keep[max_k] = 1;
    dpSplit(pts, s, max_k, thr, keep);
    dpSplit(pts, max_k, e, thr, keep);
  }
}

}  // namespace

ObstacleDetectorNode::ObstacleDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("obstacle_detector", options),
  enabled_(false)
{
  scan_topic_       = declare_parameter<std::string>("scan_topic",       "/scan");
  enable_topic_     = declare_parameter<std::string>("enable_topic",     "/obstacle_detect_enable");
  map_frame_        = declare_parameter<std::string>("map_frame",        "map");
  laser_link_frame_ = declare_parameter<std::string>("laser_link_frame", "laser_link");

  // 墙在车前方的世界 ROI（map 系，单位米），按场地标定。默认给一个宽松前方区域。
  roi_x_min_m_ = declare_parameter("roi_x_min_m",  0.20);
  roi_x_max_m_ = declare_parameter("roi_x_max_m",  6.00);
  roi_y_min_m_ = declare_parameter("roi_y_min_m", -3.00);
  roi_y_max_m_ = declare_parameter("roi_y_max_m",  3.00);

  chain_break_dist_m_  = declare_parameter("chain_break_dist_m",  0.20);
  min_chain_points_    = declare_parameter("min_chain_points",    15);
  split_threshold_m_   = declare_parameter("split_threshold_m",   0.05);
  merge_collinear_deg_ = declare_parameter("merge_collinear_deg", 10.0);
  min_total_length_m_  = declare_parameter("min_total_length_m",  0.50);
  path_corridor_half_width_m_ = declare_parameter("path_corridor_half_width_m", 0.30);

  tf_timeout_sec_ = declare_parameter("tf_timeout_sec", 0.05);

  publish_debug_points_ = declare_parameter("publish_debug_points", true);
  debug_points_topic_   = declare_parameter<std::string>("debug_points_topic", "/obstacle_debug_points");

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ObstacleDetectorNode::scanCallback, this, std::placeholders::_1));

  auto enable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  enable_sub_ = create_subscription<std_msgs::msg::Bool>(
    enable_topic_, enable_qos,
    std::bind(&ObstacleDetectorNode::enableCallback, this, std::placeholders::_1));

  target_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/target_position", enable_qos,
    std::bind(&ObstacleDetectorNode::targetCallback, this, std::placeholders::_1));

  obstacle_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/detected_obstacle",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  if (publish_debug_points_) {
    debug_points_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      debug_points_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  }

  RCLCPP_INFO(get_logger(),
    "折线障碍检测启动: scan='%s' enable='%s' frames=[%s <- %s]",
    scan_topic_.c_str(), enable_topic_.c_str(), map_frame_.c_str(), laser_link_frame_.c_str());
  RCLCPP_INFO(get_logger(),
    "ROI(map): x=[%.2f, %.2f] y=[%.2f, %.2f]  断链=%.2fm 劈开阈值=%.3fm 合并转角=%.1f° 最短总长=%.2fm",
    roi_x_min_m_, roi_x_max_m_, roi_y_min_m_, roi_y_max_m_,
    chain_break_dist_m_, split_threshold_m_, merge_collinear_deg_, min_total_length_m_);
  RCLCPP_INFO(get_logger(), "路径阻挡选择: target=/target_position corridor_half_width=%.2fm",
    path_corridor_half_width_m_);
}

// ─────────────────────────────────────────────────────────────────────────────
// /obstacle_detect_enable：边沿触发启停。使能期间每帧拟合并连续发布。
// ─────────────────────────────────────────────────────────────────────────────
void ObstacleDetectorNode::enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->data == enabled_) { return; }
  enabled_ = msg->data;
  RCLCPP_INFO(get_logger(), enabled_ ? "检测使能：开始逐帧拟合..." : "检测禁用。");
}

void ObstacleDetectorNode::targetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->data.size() < 4) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "target_position 需要 [x_cm, y_cm, z_cm, yaw_deg]");
    has_ground_target_ = false;
    return;
  }
  if (msg->data[2] > 20.0F) {
    has_ground_target_ = false;
    return;
  }
  ground_target_ = {static_cast<double>(msg->data[0]) / 100.0,
    static_cast<double>(msg->data[1]) / 100.0};
  has_ground_target_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 单帧：用 TF 把每个点变到 map 系（保持扫描序），再用 ROI 过滤。
// 同时取出车在 map 系的位置。
// ─────────────────────────────────────────────────────────────────────────────
std::vector<Pt2> ObstacleDetectorNode::collectRoiPoints(
  const sensor_msgs::msg::LaserScan & scan, Pt2 & car_xy_out)
{
  std::vector<Pt2> pts;
  const int n = static_cast<int>(scan.ranges.size());
  if (n < 8) { return pts; }

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, scan.header.stamp,
      rclcpp::Duration::from_seconds(tf_timeout_sec_));
  } catch (const tf2::TransformException &) {
    try {
      tf_msg = tf_buffer_->lookupTransform(map_frame_, laser_link_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s <- %s 查询失败: %s", map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
      return pts;
    }
  }

  tf2::Transform T_ml;
  tf2::fromMsg(tf_msg.transform, T_ml);

  const tf2::Vector3 car = T_ml * tf2::Vector3(0.0, 0.0, 0.0);
  car_xy_out = {car.x(), car.y()};

  pts.reserve(static_cast<std::size_t>(n) / 2);
  const double ang_min = static_cast<double>(scan.angle_min);
  const double ang_res = static_cast<double>(scan.angle_increment);

  for (int i = 0; i < n; ++i) {
    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) { continue; }

    const double theta = ang_min + static_cast<double>(i) * ang_res;
    const double xl = static_cast<double>(r) * std::cos(theta);
    const double yl = static_cast<double>(r) * std::sin(theta);

    const tf2::Vector3 p_map = T_ml * tf2::Vector3(xl, yl, 0.0);
    const double xm = p_map.x();
    const double ym = p_map.y();

    if (xm >= roi_x_min_m_ && xm <= roi_x_max_m_ &&
        ym >= roi_y_min_m_ && ym <= roi_y_max_m_)
    {
      pts.push_back({xm, ym});
    }
  }

  if (publish_debug_points_ && debug_points_pub_ && !pts.empty()) {
    std_msgs::msg::Float32MultiArray dbg;
    dbg.data.reserve(pts.size() * 2);
    for (const auto & p : pts) {
      dbg.data.push_back(static_cast<float>(p.x));
      dbg.data.push_back(static_cast<float>(p.y));
    }
    debug_points_pub_->publish(dbg);
  }

  return pts;
}

// ─────────────────────────────────────────────────────────────────────────────
// 按相邻点距离断链，保留所有候选墙体。
// ─────────────────────────────────────────────────────────────────────────────
std::vector<std::vector<Pt2>> ObstacleDetectorNode::splitChains(const std::vector<Pt2> & pts) const
{
  std::vector<std::vector<Pt2>> chains;
  std::vector<Pt2> current;
  for (const auto & point : pts) {
    if (!current.empty() &&
      std::hypot(point.x - current.back().x, point.y - current.back().y) > chain_break_dist_m_)
    {
      chains.push_back(std::move(current));
      current.clear();
    }
    current.push_back(point);
  }
  if (!current.empty()) { chains.push_back(std::move(current)); }
  return chains;
}

// ─────────────────────────────────────────────────────────────────────────────
// Split-and-Merge：先 Douglas–Peucker 取顶点，再按转角合并近共线段。
// ─────────────────────────────────────────────────────────────────────────────
Polyline ObstacleDetectorNode::fitPolyline(const std::vector<Pt2> & chain) const
{
  Polyline out;
  const int n = static_cast<int>(chain.size());
  if (n < min_chain_points_ || n < 2) { return out; }

  // ── Split：标记保留顶点（首尾必留） ──
  std::vector<char> keep(n, 0);
  keep[0] = 1;
  keep[n - 1] = 1;
  dpSplit(chain, 0, n - 1, split_threshold_m_, keep);

  std::vector<Pt2> verts;
  for (int i = 0; i < n; ++i) {
    if (keep[i]) { verts.push_back(chain[i]); }
  }

  // ── Merge：删掉转角太小（近共线）的中间顶点 ──
  const double merge_cos = std::cos(merge_collinear_deg_ * M_PI / 180.0);
  bool changed = true;
  while (changed && verts.size() > 2) {
    changed = false;
    for (std::size_t i = 1; i + 1 < verts.size(); ++i) {
      const double ax = verts[i].x - verts[i - 1].x, ay = verts[i].y - verts[i - 1].y;
      const double bx = verts[i + 1].x - verts[i].x, by = verts[i + 1].y - verts[i].y;
      const double la = std::hypot(ax, ay), lb = std::hypot(bx, by);
      if (la < 1e-9 || lb < 1e-9) { verts.erase(verts.begin() + i); changed = true; break; }
      const double cosang = (ax * bx + ay * by) / (la * lb);
      if (cosang >= merge_cos) {   // 转角 < 阈值 → 近共线，删中间点
        verts.erase(verts.begin() + i);
        changed = true;
        break;
      }
    }
  }

  if (verts.size() < 2) { return out; }

  // ── 总长 ──
  double total = 0.0;
  for (std::size_t i = 1; i < verts.size(); ++i) {
    total += std::hypot(verts[i].x - verts[i - 1].x, verts[i].y - verts[i - 1].y);
  }
  if (total < min_total_length_m_) { return out; }

  out.valid = true;
  out.vertices = std::move(verts);
  out.total_length = total;
  return out;
}

bool ObstacleDetectorNode::pathDistanceToPolyline(
  const Pt2 & car_xy, const Pt2 & target_xy, const Polyline & poly, double & path_dist) const
{
  const double route_length = std::hypot(target_xy.x - car_xy.x, target_xy.y - car_xy.y);
  if (route_length < 1e-3) { return false; }

  bool blocks_path = false;
  double nearest_path_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 1; i < poly.vertices.size(); ++i) {
    double corridor_dist = 0.0;
    double route_t = 0.0;
    if (!segmentPathDistance(
        car_xy, target_xy, poly.vertices[i - 1], poly.vertices[i], corridor_dist, route_t))
    {
      continue;
    }
    if (corridor_dist <= path_corridor_half_width_m_ && route_t > 1e-3) {
      blocks_path = true;
      nearest_path_dist = std::min(nearest_path_dist, route_t * route_length);
    }
  }
  if (blocks_path) { path_dist = nearest_path_dist; }
  return blocks_path;
}

void ObstacleDetectorNode::publishObstacle(const Polyline & poly)
{
  // 输出布局（map 系，单位 m）：
  //   [0] N 顶点数
  //   [1 .. 2N] 顶点 x0,y0, x1,y1, ..., x(N-1),y(N-1)
  //   [2N+1] path_dist 沿当前路线到阻挡墙的距离
  //   [2N+2] total_length 折线总长
  const int N = static_cast<int>(poly.vertices.size());
  std_msgs::msg::Float32MultiArray msg;
  msg.data.reserve(static_cast<std::size_t>(2 * N + 3));
  msg.data.push_back(static_cast<float>(N));
  for (const auto & v : poly.vertices) {
    msg.data.push_back(static_cast<float>(v.x));
    msg.data.push_back(static_cast<float>(v.y));
  }
  msg.data.push_back(static_cast<float>(poly.path_dist));
  msg.data.push_back(static_cast<float>(poly.total_length));
  obstacle_pub_->publish(msg);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "[路线阻挡墙] %d 顶点(%d 段) 路线距离%.2fm 总长%.2fm 首(%.2f,%.2f) 尾(%.2f,%.2f)",
    N, N - 1, poly.path_dist, poly.total_length,
    poly.vertices.front().x, poly.vertices.front().y,
    poly.vertices.back().x, poly.vertices.back().y);
}

void ObstacleDetectorNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) { return; }
  if (!has_ground_target_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "等待当前地面航点 /target_position，不执行阻挡墙选择");
    return;
  }

  Pt2 car{0.0, 0.0};
  const auto pts = collectRoiPoints(*msg, car);
  if (pts.empty()) { return; }

  const auto chains = splitChains(pts);
  Polyline selected;
  double nearest_path_dist = std::numeric_limits<double>::max();
  std::size_t valid_count = 0;
  for (const auto & chain : chains) {
    Polyline poly = fitPolyline(chain);
    if (!poly.valid) { continue; }
    ++valid_count;
    double path_dist = 0.0;
    if (pathDistanceToPolyline(car, ground_target_, poly, path_dist) &&
      path_dist < nearest_path_dist)
    {
      nearest_path_dist = path_dist;
      poly.path_dist = path_dist;
      selected = std::move(poly);
    }
  }

  if (!selected.valid) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "ROI 内 %zu 点、%zu 条链、%zu 条有效墙，当前路线未被阻挡",
      pts.size(), chains.size(), valid_count);
    return;
  }
  publishObstacle(selected);
}

}  // namespace obstacle_detector_pkg
