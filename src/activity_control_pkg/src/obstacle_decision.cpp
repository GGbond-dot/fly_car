#include "activity_control_pkg/obstacle_decision.hpp"

#include <cmath>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

ObstacleDecisionNode::ObstacleDecisionNode(
  std::shared_ptr<RouteTargetPublisherNode> route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("obstacle_decision", options),
  route_node_(std::move(route_node))
{
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  approach_threshold_m_ = declare_parameter("approach_threshold_m", 0.6);  // 以车为心半径 0.6m
  flyover_z_cm_ = declare_parameter("flyover_z_cm", 100.0);                // 越障飞行高度
  single_shot_ = declare_parameter("single_shot", true);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // /obstacle_detect_enable 与 obstacle_detector 约定 transient_local(边沿使能)
  detect_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/obstacle_detect_enable",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  obstacle_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/detected_obstacle", rclcpp::QoS(10),
    std::bind(&ObstacleDecisionNode::obstacleCallback, this, std::placeholders::_1));

  // 启动即开检测(地面遍历期间持续输出阻挡墙)
  publishDetectEnable(true);

  RCLCPP_INFO(get_logger(),
    "obstacle_decision: 触发阈值=%.2fm 越障高度=%.0fcm single_shot=%d",
    approach_threshold_m_, flyover_z_cm_, single_shot_);
}

void ObstacleDecisionNode::publishDetectEnable(bool on)
{
  std_msgs::msg::Bool msg;
  msg.data = on;
  detect_enable_pub_->publish(msg);
}

bool ObstacleDecisionNode::getCarPose(double & x_m, double & y_m, double & yaw_rad) const
{
  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_m = tf.transform.translation.x;
    y_m = tf.transform.translation.y;
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_rad = yaw;
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void ObstacleDecisionNode::obstacleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (done_ && single_shot_) {
    return;
  }

  // 解析 /detected_obstacle:[N, x0,y0,...,x(N-1),y(N-1), path_dist, total_length]
  const auto & d = msg->data;
  if (d.empty()) {
    return;
  }
  const std::size_t n = static_cast<std::size_t>(d[0]);
  if (n < 2 || d.size() < 1 + 2 * n + 2) {
    return;
  }
  const double path_dist = d[1 + 2 * n];
  if (path_dist > approach_threshold_m_) {
    return;  // 还没逼近到触发半径
  }

  // 当前自身位姿:原地起飞点取这里(垂直拔高,xy 不动)
  double car_x = 0.0, car_y = 0.0, car_yaw = 0.0;
  if (!getCarPose(car_x, car_y, car_yaw)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "obstacle_decision: 触发但无自身 TF,暂不起飞");
    return;
  }

  // 1. 插“当前 xy + z=flyover”航点 → 原地垂直起飞
  const double yaw_deg = car_yaw * 180.0 / M_PI;
  std::vector<Target> takeoff = {
    Target{car_x * 100.0, car_y * 100.0, flyover_z_cm_, yaw_deg},
  };
  route_node_->insertNext(takeoff);

  // 2. 全局 z 覆盖 → 后续原 xy 航点都在 flyover 高度飞越(沿原路线)
  route_node_->setFlightMode(true, flyover_z_cm_);

  done_ = true;
  publishDetectEnable(false);  // 任务只有这一道墙,触发后关检测

  RCLCPP_INFO(get_logger(),
    "墙逼近 path_dist=%.2fm ≤ %.2fm:原地起飞(%.2f,%.2f) z=%.0fcm,转飞行模式沿原 xy 飞越",
    path_dist, approach_threshold_m_, car_x, car_y, flyover_z_cm_);
}

}  // namespace activity_control_pkg
