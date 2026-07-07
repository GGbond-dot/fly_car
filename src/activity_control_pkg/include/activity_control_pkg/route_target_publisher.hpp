#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

struct Target
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
  bool land_after = false;  // 标志位:飞到此航点后原地垂直下降回地面(用户在落点航点上打)
};

class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  explicit RouteTargetPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void addTarget(const Target & target);

  // 在“当前正在追的航点”之前原子插入一串航点(障碍决策插“原地起飞点”用)。
  // 当前目标被往后顶,先飞越再继续原目标;队列尚未开始时等同依次 addTarget。
  void insertNext(const std::vector<Target> & batch);

  // 全局 z 覆盖(飞行模式):active 时发布/到达判定都把 z 顶成 flight_z_cm,
  // xy/yaw 仍用原航点 —— 即“沿原来的 xy 在空中飞”。关掉则恢复各航点自身 z。
  void setFlightMode(bool active, double flight_z_cm = 100.0);

  std::size_t currentIndex() const;

  std::size_t size() const;

private:
  void publishCurrent(bool verbose = true);  // verbose=false:心跳静默重发,不打日志
  void publishTarget(const Target & target, bool init_flag, bool verbose);
  Target effectiveTarget(const Target & t) const;  // 应用飞行模式 z 覆盖

  bool getCurrentPose(double & x_cm, double & y_cm, double & z_cm, double & yaw_deg);
  bool isReached(const Target & target, double x_cm, double y_cm, double z_cm, double yaw_deg) const;

  void monitorTimerCallback();
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void is_st_ready_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  
  static double meterToCm(double value_m);
  static double radToDeg(double value_rad);
  double normalizeAngleDeg(double angle_deg) const;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr is_st_ready_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  std::vector<Target> targets_;
  std::size_t current_idx_;
  std::size_t lookahead_count_{0};  // /target_position 后追加的前视航点数(0=不追加)

  bool flight_mode_ = false;     // 全局 z 覆盖开关
  double flight_z_cm_ = 100.0;   // 覆盖高度

  bool has_height_;
  double current_height_cm_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;
  double ground_z_tol_cm_;  // 地面航点 z 容忍(松):忽略地面噪声,但仍能拦住“还没真正降到地面”
  double air_z_tol_cm_;     // 空中航点 z 容忍(紧):高度要到位
  double land_z_cm_;        // land_after 落点的地面高度

  std::string map_frame_;
  std::string laser_link_frame_;
  std::string output_topic_;

  bool ever_received_st_ready_ = false;   // 只要收到过 1 就永远 true
};

class RouteTestNode : public rclcpp::Node
{
public:
  explicit RouteTestNode(const std::shared_ptr<RouteTargetPublisherNode> & route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::shared_ptr<RouteTargetPublisherNode> route_node_;
};

}  // namespace activity_control_pkg
