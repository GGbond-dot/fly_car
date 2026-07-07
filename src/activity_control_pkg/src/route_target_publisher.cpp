#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>
#include <clocale>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

namespace
{
constexpr double kDefaultTimerPeriodSec = 0.05;
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_idx_(std::numeric_limits<std::size_t>::max()),
  has_height_(false),
  current_height_cm_(0.0)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  ground_z_tol_cm_ = declare_parameter("ground_z_tol_cm", 30.0);  // 地面:松(大)
  air_z_tol_cm_ = declare_parameter("air_z_tol_cm", 8.0);         // 空中:紧(小)
  land_z_cm_ = declare_parameter("land_z_cm", 4.0);              // land_after 落点高度
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");
  // pure-pursuit 前视:>0 时在 /target_position 消息后追加接下来 N 个航点的 xy(cm),供控制器取前视点。
  // 默认 0=不追加,消息仍是 [x,y,z,yaw] 4 位,老订阅者(chassis_mux 等)完全兼容。
  lookahead_count_ = static_cast<std::size_t>(declare_parameter("lookahead_count", 0));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, qos);

  is_st_ready_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/is_st_ready", 10, std::bind(&RouteTargetPublisherNode::is_st_ready_callback, this, std::placeholders::_1));

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  RCLCPP_INFO(get_logger(),
    "RouteTargetPublisher initialized: map=%s laser_link=%s topic=%s", map_frame_.c_str(),
    laser_link_frame_.c_str(), output_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm",
    pos_tol_cm_, yaw_tol_deg_, height_tol_cm_);
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  targets_.push_back(target);
  if (was_empty) {
    current_idx_ = 0;
    publishCurrent();
  }
}

void RouteTargetPublisherNode::insertNext(const std::vector<Target> & batch)
{
  if (batch.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // 队列还没开始(无当前目标):退化为依次追加
  if (current_idx_ == std::numeric_limits<std::size_t>::max() || targets_.empty()) {
    const bool was_empty = targets_.empty();
    targets_.insert(targets_.end(), batch.begin(), batch.end());
    if (was_empty) {
      current_idx_ = 0;
      publishCurrent();
    }
    return;
  }

  // 所有目标已完成(current_idx_ 越界)时,从队尾接着追加并重启推进
  if (current_idx_ >= targets_.size()) {
    current_idx_ = targets_.size();
    targets_.insert(targets_.end(), batch.begin(), batch.end());
    publishCurrent();
    return;
  }

  // 正常情况:在当前目标之前插入 batch。current_idx_ 数值不变,
  // 但现在指向 batch 的第一个(起飞点);原当前目标顺延到 batch 之后。
  targets_.insert(targets_.begin() + static_cast<std::ptrdiff_t>(current_idx_),
    batch.begin(), batch.end());
  RCLCPP_INFO(get_logger(),
    "insertNext: 在 idx=%zu 前插入 %zu 个航点(越障序列),队列总数 %zu",
    current_idx_, batch.size(), targets_.size());
  publishCurrent();
}

std::size_t RouteTargetPublisherNode::currentIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

std::size_t RouteTargetPublisherNode::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

void RouteTargetPublisherNode::setFlightMode(bool active, double flight_z_cm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  flight_mode_ = active;
  flight_z_cm_ = flight_z_cm;
  RCLCPP_INFO(get_logger(),
    "setFlightMode: %s (z 覆盖=%.0fcm) —— 后续航点沿原 xy 在空中飞",
    active ? "ON" : "OFF", flight_z_cm_);
  publishCurrent();  // 立即按新 z 重发当前目标,chassis_mux 据此切换
}

// 飞行模式下把 z 顶成 flight_z_cm,xy/yaw 不变;否则原样
Target RouteTargetPublisherNode::effectiveTarget(const Target & t) const
{
  if (!flight_mode_) {
    return t;
  }
  Target e = t;
  e.z_cm = flight_z_cm_;
  return e;
}

void RouteTargetPublisherNode::publishCurrent(bool verbose)
{
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ < targets_.size()) {
    publishTarget(effectiveTarget(targets_[current_idx_]), current_idx_ == 0, verbose);
  }
}

void RouteTargetPublisherNode::publishTarget(const Target & target, bool init_flag, bool verbose)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = static_cast<float>(target.yaw_deg);
  // 追加前视航点 xy(cm):当前点之后的 lookahead_count_ 个。控制器据此取前方前视点、自动圆角。
  for (std::size_t k = 1; k <= lookahead_count_ &&
       current_idx_ != std::numeric_limits<std::size_t>::max() &&
       current_idx_ + k < targets_.size(); ++k) {
    message.data.push_back(static_cast<float>(targets_[current_idx_ + k].x_cm));
    message.data.push_back(static_cast<float>(targets_[current_idx_ + k].y_cm));
  }
  target_pub_->publish(message);
  if (verbose) {
    RCLCPP_INFO(get_logger(),
      "发布目标: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg%s",
      target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
      init_flag ? " (首个)" : "");
  }
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

bool RouteTargetPublisherNode::getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    z_cm = has_height_ ? current_height_cm_ : 0.0;
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "TF 查询失败 (%s->%s): %s", map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

void RouteTargetPublisherNode::is_st_ready_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data == 1 && !ever_received_st_ready_) {
    ever_received_st_ready_ = true;   // 永久置位
    RCLCPP_INFO(this->get_logger(), "收到 /is_st_ready=1，进入飞机模式");
  }
}


bool RouteTargetPublisherNode::isReached(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dxy = std::hypot(dx, dy);
  const double dz = target.z_cm - z_cm;
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);

  // 空中航点(z>20):z 容忍紧(高度必须到位);地面航点:z 容忍松(忽略地面噪声,
  // 但松到的有限值仍能拦住“从空中降回地面前就误判到达”)。
  const bool airborne = target.z_cm > 20.0;
  const double z_tol = airborne ? air_z_tol_cm_ : ground_z_tol_cm_;
  const bool z_ok = (std::fabs(dz) <= z_tol);
  const bool xy_ok = (dxy <= pos_tol_cm_);
  const bool yaw_ok = (std::fabs(dyaw) <= yaw_tol_deg_);

  if (airborne) {
    // 空中航点:高度到位 + xy 到位,放宽 yaw
    return z_ok && xy_ok;
  }
  // 地面航点:xy + yaw + (松)z 都要满足
  return z_ok && xy_ok && yaw_ok;
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_idx_ == std::numeric_limits<std::size_t>::max() || current_idx_ >= targets_.size()) {
    return;
  }

  // 心跳:每个 tick 重发当前目标(静默,不打日志)。/target_position 虽是锁存单发,
  // 但下游 diff_drive_controller 有 target_timeout_s 安全超时——没有持续心跳它会在 ~2s 后
  // 判目标过期并永久沉默(车停死)。周期重发让它一直追,同时保留"发布器一挂心跳停、
  // 控制器安全停车"的语义。
  publishCurrent(/*verbose=*/false);

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  const Target target = effectiveTarget(targets_[current_idx_]);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "当前目标 %zu: x=%.1f,y=%.1f,z=%.1f,yaw=%.1f",
    current_idx_, target.x_cm, target.y_cm, target.z_cm, target.yaw_deg
  );
  if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
    const Target reached_orig = targets_[current_idx_];  // 原航点(含 land_after 标志与原 xy)
    RCLCPP_INFO(get_logger(),
      "目标 %zu 已完成，准备下一个", current_idx_);
    current_idx_++;

    // 带 land_after 标志且当前在飞:到达后原地垂直下降回地面。
    // 退出 z 覆盖(落点用真实低 z),插一个同 xy 的下降航点;已持锁,直接置 flight_mode_。
    if (reached_orig.land_after && flight_mode_) {
      flight_mode_ = false;
      Target descent{reached_orig.x_cm, reached_orig.y_cm, land_z_cm_, reached_orig.yaw_deg, false};
      targets_.insert(targets_.begin() + static_cast<std::ptrdiff_t>(current_idx_), descent);
      RCLCPP_INFO(get_logger(),
        "到达落点航点(%.1f,%.1f):退出飞行模式,插原地下降点 z=%.0fcm",
        reached_orig.x_cm, reached_orig.y_cm, land_z_cm_);
    }

    if (current_idx_ < targets_.size()) {
      publishCurrent();
    } else {
      current_idx_ = targets_.size();
      RCLCPP_INFO(get_logger(), "所有目标已完成");
    }
  }
}

double RouteTargetPublisherNode::meterToCm(double value_m)
{
  return value_m * 100.0;
}

double RouteTargetPublisherNode::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg) const
{
  const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(normalized);
}


RouteTestNode::RouteTestNode(
  const std::shared_ptr<RouteTargetPublisherNode> & route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("route_test_node", options),
  route_node_(route_node)
{
  std::setlocale(LC_ALL, "");

  // 航点走参数,便于 launch 配置不同测试(跑方形/飞方形)而不必改代码重编。
  // 扁平数组每 4 个一组 [x_cm, y_cm, z_cm, yaw_deg],与 /target_position 布局一致。
  // 默认沿用原演示序列(先前进 2m,升到 100cm 飞方形,再降落),保证 demo1 行为不变。
  const std::vector<double> default_wp{
    200.0, 0.0, 4.0, 0.0,
    200.0, 0.0, 100.0, 0.0,
    200.0, 200.0, 100.0, 0.0,
    0.0, 200.0, 100.0, 0.0,
    0.0, 200.0, 0.0, 0.0};
  const auto flat = declare_parameter<std::vector<double>>("waypoints", default_wp);

  if (flat.size() < 4 || flat.size() % 4 != 0) {
    RCLCPP_FATAL(get_logger(),
      "waypoints 必须是 4 的倍数 [x_cm,y_cm,z_cm,yaw_deg,...],当前 %zu 个数", flat.size());
    throw std::runtime_error("invalid waypoints parameter");
  }

  // 一次性把全部航点压入队列;RouteTargetPublisher 按到达自动推进。
  for (std::size_t i = 0; i + 3 < flat.size(); i += 4) {
    const Target t{flat[i], flat[i + 1], flat[i + 2], flat[i + 3]};
    route_node_->addTarget(t);
    RCLCPP_INFO(get_logger(),
      "添加航点 %zu: x=%.1f y=%.1f z=%.1f yaw=%.1f",
      i / 4, t.x_cm, t.y_cm, t.z_cm, t.yaw_deg);
  }

  RCLCPP_INFO(get_logger(), "Route test node 启动,共加载 %zu 个航点。", flat.size() / 4);
}

}  // namespace activity_control_pkg
