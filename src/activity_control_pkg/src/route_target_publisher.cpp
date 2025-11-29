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
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");

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

void RouteTargetPublisherNode::publishCurrent()
{
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ < targets_.size()) {
    publishTarget(targets_[current_idx_], current_idx_ == 0);
  }
}

void RouteTargetPublisherNode::publishTarget(const Target & target, bool init_flag)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = static_cast<float>(target.yaw_deg);
  target_pub_->publish(message);
  RCLCPP_INFO(get_logger(),
    "发布目标: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg%s",
    target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
    init_flag ? " (首个)" : "");
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

// bool RouteTargetPublisherNode::isReached(
//   const Target & target,
//   double x_cm,
//   double y_cm,
//   double z_cm,
//   double yaw_deg) const
// {
//   const double dx = target.x_cm - x_cm;
//   const double dy = target.y_cm - y_cm;
//   const double dxy = std::hypot(dx, dy);
//   const double dz = target.z_cm - z_cm;
//   const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);
//   // 容忍度动态选择
//   const double z_tol = ever_received_st_ready_ ? height_tol_cm_ : 20.0;            // 飞机： 6 cm   车：20 cm 
//   const bool z_ok = (std::fabs(dz) <= z_tol);
//   // const bool z_ok = has_height_ ? (std::fabs(dz) <= height_tol_cm_) : true;
//   return z_ok && (dxy <= pos_tol_cm_) && (std::fabs(dyaw) <= yaw_tol_deg_);
// }

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
  
  const double z_tol = ever_received_st_ready_ ? height_tol_cm_ : 20.0;
  const bool z_ok = (std::fabs(dz) <= z_tol);
  const bool xy_ok = (dxy <= pos_tol_cm_);
  const bool yaw_ok = (std::fabs(dyaw) <= yaw_tol_deg_);

  // 如果目标Z值大于一个阈值（比如20cm），说明这是一个起飞或空中航点
  // 这种情况下，我们放宽对XY和Yaw的要求，只要高度差不多就认为到达
  if (target.z_cm > 20.0) {
    // 对于起飞阶段，主要关心高度是否到达
    if (current_idx_ == 0) {
        return z_ok;
    }
    // 对于空中的航点，只要高度和水平位置都差不多就行，暂时忽略yaw
    return z_ok && xy_ok;
  }

  // 对于Z值很低（比如降落）或为0的航点，要求所有条件都满足
  return z_ok && xy_ok && yaw_ok;
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_idx_ == std::numeric_limits<std::size_t>::max() || current_idx_ >= targets_.size()) {
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  const Target & target = targets_[current_idx_];
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "当前目标 %zu: x=%.1f,y=%.1f,z=%.1f,yaw=%.1f",
    current_idx_, target.x_cm, target.y_cm, target.z_cm, target.yaw_deg
  );
  if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
    RCLCPP_INFO(get_logger(),
      "目标 %zu 已完成，准备下一个", current_idx_);
    current_idx_++;
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

// RouteTestNode::RouteTestNode(
//   const std::shared_ptr<RouteTargetPublisherNode> & route_node,
//   const rclcpp::NodeOptions & options)
// : rclcpp::Node("route_test_node", options),
//   route_node_(route_node),
//   started_(false),
//   next_target_index_(1)
// {
//   std::setlocale(LC_ALL, "");

//   ready_sub_ = create_subscription<std_msgs::msg::UInt8>(
//     "/is_st_ready", rclcpp::QoS(10),
//     std::bind(&RouteTestNode::readyCallback, this, std::placeholders::_1));

//   add_timer_ = create_wall_timer(
//     std::chrono::seconds(1),
//     std::bind(&RouteTestNode::addTimerCallback, this));
//   add_timer_->cancel();

//   RCLCPP_INFO(get_logger(), "Route test node ready. 等待 /is_st_ready == 1");
// }

// void RouteTestNode::readyCallback(const std_msgs::msg::UInt8::SharedPtr msg)
// {
//   if (msg->data == 1 && !started_) {
//     Target first{0.0, 0.0, 140, 0.0};
//     route_node_->addTarget(first);
//     const auto current = route_node_->currentIndex();
//     RCLCPP_INFO(get_logger(),
//       "收到 /is_st_ready=1，添加首个目标: x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
//       first.x_cm, first.y_cm, first.z_cm, first.yaw_deg,
//       (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));
//     add_timer_->reset();
//     started_ = true;
//   } else if (!started_) {
//     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
//       "/is_st_ready=%u，等待为1", static_cast<unsigned>(msg->data));
//   }
// }

RouteTestNode::RouteTestNode(
  const std::shared_ptr<RouteTargetPublisherNode> & route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("route_test_node", options),
  route_node_(route_node),
  started_(false), // started_ 标志仍然有用，但现在在构造函数中设置
  next_target_index_(1)
{
  std::setlocale(LC_ALL, "");

  // 1. 移除 /is_st_ready 的订阅
  // ready_sub_ = create_subscription<std_msgs::msg::UInt8>(...); // 此行已被删除

  // 创建定时器，但先不启动
  add_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&RouteTestNode::addTimerCallback, this));
  add_timer_->cancel();

  // 2. 直接执行添加第一个目标点的逻辑
  RCLCPP_INFO(get_logger(), "Route test node 启动，自动添加首个目标。");
  
  // Target first{0.0, 0.0, 160, 0.0};
  Target first{200.0, 0.0, 4.0, 0.0};
  route_node_->addTarget(first);
  
  // 3. 保留并更新日志信息
  const auto current = route_node_->currentIndex();
  RCLCPP_INFO(get_logger(),
    "添加首个目标: x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
    first.x_cm, first.y_cm, first.z_cm, first.yaw_deg,
    (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));
  
  // 4. 启动定时器以添加后续目标，并设置标志位
  add_timer_->reset();
  started_ = true;
}




void RouteTestNode::addTimerCallback()
{
  if (!started_) {
    return;
  }

  Target target{};
  switch (next_target_index_) {
    case 1:
      target = Target{200.0, 0.0, 100.0, 0.0};
      break;
    case 2:
      target = Target{200.0, 200.0, 100.0, 0.0};
      break;
    case 3:
      target = Target{0.0, 200.0, 100.0, 0.0};
      break;
    case 4:
      target  = Target{0.0, 200.0, 0.0, 0.0};
      break;
    default:
      add_timer_->cancel();
      RCLCPP_INFO(get_logger(), "预设目标全部添加完毕");
      return;
  }

  route_node_->addTarget(target);
  const auto current = route_node_->currentIndex();
  RCLCPP_INFO(get_logger(),
    "追加目标 idx=%d: x=%.1f y=%.1f z=%.1f yaw=%.1f | 当前第 %zu 个目标",
    next_target_index_, target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
    (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));

  ++next_target_index_;
}

}  // namespace activity_control_pkg
