// diff_drive_controller — 飞车地面差速跟踪控制器
//
// 仿照 car/follower_pkg/diff_drive_controller.cpp(同款 SR5E1E3 差速底盘,不能横移),
// 增加 /ground_enable 使能门:只有地面态(chassis_mux 使能)才驱动轮子,
// 保证与飞控 PID 互斥(车跑时飞控不动,反之亦然)。
//
//   输入  /target_position(Float32MultiArray [x_cm, y_cm, z_cm, yaw_deg],map 系移动靶)
//         /ground_enable(Bool,chassis_mux 按目标 z 仲裁)
//         TF map←laser_link(自身位姿)
//   输出  /cmd_vel(Twist,linear.x=v m/s, angular.z=w rad/s)→ chassis_bridge 转 $VW
//
// 控制律(carrot-chasing):同 car 版,先对准方位再前进。
// 使能门:!ground_enable 时持续发零速(轮子停),不跑控制律。

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}
}  // namespace

class DiffDriveController : public rclcpp::Node
{
public:
  DiffDriveController()
  : Node("diff_drive_controller")
  {
    declare_parameter<double>("kp_v", 1.0);             // v = kp_v * 距离误差(m)
    declare_parameter<double>("v_max_mps", 0.4);
    declare_parameter<double>("kp_w", 1.5);             // w = kp_w * 角度误差(rad)
    declare_parameter<double>("w_max_rps", 1.0);
    declare_parameter<double>("align_gate_deg", 45.0);  // 方位误差超过此值先原地转向
    declare_parameter<double>("pos_tol_cm", 5.0);
    declare_parameter<double>("yaw_tol_deg", 8.0);
    declare_parameter<double>("target_timeout_s", 1.0);
    declare_parameter<double>("stop_burst_s", 1.0);     // 超时后零速发送时长
    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<bool>("ground_enable_default", false);  // 无 mux 时的默认态(安全起见默认禁用)

    kp_v_ = get_parameter("kp_v").as_double();
    v_max_ = get_parameter("v_max_mps").as_double();
    kp_w_ = get_parameter("kp_w").as_double();
    w_max_ = get_parameter("w_max_rps").as_double();
    align_gate_rad_ = get_parameter("align_gate_deg").as_double() * M_PI / 180.0;
    pos_tol_m_ = get_parameter("pos_tol_cm").as_double() / 100.0;
    yaw_tol_rad_ = get_parameter("yaw_tol_deg").as_double() * M_PI / 180.0;
    target_timeout_s_ = get_parameter("target_timeout_s").as_double();
    stop_burst_s_ = get_parameter("stop_burst_s").as_double();
    ground_enabled_ = get_parameter("ground_enable_default").as_bool();
    const double rate_hz = get_parameter("publish_rate_hz").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    target_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_position", rclcpp::QoS(10),
      std::bind(&DiffDriveController::targetCallback, this, std::placeholders::_1));

    // 使能门:latched(transient_local),晚启动也能拿到 mux 当前态
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/ground_enable", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&DiffDriveController::enableCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));

    const double period_sec = 1.0 / std::max(rate_hz, 1.0);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec)),
      std::bind(&DiffDriveController::controlTimerCallback, this));

    RCLCPP_INFO(get_logger(),
      "diff_drive_controller up (kp_v=%.2f v_max=%.2f kp_w=%.2f w_max=%.2f gate=%.0fdeg) ground_enable=%d",
      kp_v_, v_max_, kp_w_, w_max_, align_gate_rad_ * 180.0 / M_PI, ground_enabled_);
  }

private:
  void targetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "target_position requires 4 floats [x_cm, y_cm, z_cm, yaw_deg]");
      return;
    }
    target_x_m_ = static_cast<double>(msg->data[0]) / 100.0;
    target_y_m_ = static_cast<double>(msg->data[1]) / 100.0;
    target_yaw_rad_ = static_cast<double>(msg->data[3]) * M_PI / 180.0;
    last_target_time_ = now();
    has_target_ = true;
  }

  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data != ground_enabled_) {
      RCLCPP_INFO(get_logger(), "ground_enable -> %s", msg->data ? "true(地面态)" : "false(飞控态/停)");
    }
    ground_enabled_ = msg->data;
  }

  bool getCurrentPose(double & x, double & y, double & yaw)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform("map", "laser_link", tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;

      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF map->laser_link unavailable: %s", ex.what());
      return false;
    }
  }

  void publishCmd(double v, double w)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = v;
    msg.angular.z = w;
    cmd_vel_pub_->publish(msg);
  }

  void controlTimerCallback()
  {
    // 使能门:非地面态 → 持续发零速(轮子停),不跑控制律,保证与飞控互斥
    if (!ground_enabled_) {
      publishCmd(0.0, 0.0);
      return;
    }

    if (!has_target_) {
      return;  // 从未收到目标:保持沉默,不抢 /cmd_vel
    }

    // 目标超时:零速刹停 stop_burst_s,然后转入沉默,等新目标
    const double age_s = (now() - last_target_time_).seconds();
    if (age_s > target_timeout_s_) {
      if (age_s <= target_timeout_s_ + stop_burst_s_) {
        publishCmd(0.0, 0.0);
      } else if (!silenced_) {
        silenced_ = true;
        RCLCPP_WARN(get_logger(), "target stale %.1fs -> stop burst done, going silent", age_s);
      }
      return;
    }
    silenced_ = false;

    double x, y, yaw;
    if (!getCurrentPose(x, y, yaw)) {
      publishCmd(0.0, 0.0);  // 有目标但无定位:宁可停车
      return;
    }

    const double dx = target_x_m_ - x;
    const double dy = target_y_m_ - y;
    const double d = std::hypot(dx, dy);

    double v = 0.0;
    double w = 0.0;

    if (d > pos_tol_m_) {
      // 追位置:先对准目标点方位,再前进
      const double e_h = normalizeAngle(std::atan2(dy, dx) - yaw);
      w = clamp(kp_w_ * e_h, -w_max_, w_max_);
      if (std::fabs(e_h) <= align_gate_rad_) {
        v = clamp(kp_v_ * d, 0.0, v_max_) * std::cos(e_h);
      }
    } else {
      // 位置到了:原地对准目标 yaw
      const double e_yaw = normalizeAngle(target_yaw_rad_ - yaw);
      if (std::fabs(e_yaw) > yaw_tol_rad_) {
        w = clamp(kp_w_ * e_yaw, -w_max_, w_max_);
      }
    }

    publishCmd(v, w);
  }

  // 参数
  double kp_v_, v_max_, kp_w_, w_max_;
  double align_gate_rad_, pos_tol_m_, yaw_tol_rad_;
  double target_timeout_s_, stop_burst_s_;

  // 状态
  double target_x_m_{0.0}, target_y_m_{0.0}, target_yaw_rad_{0.0};
  bool has_target_{false};
  bool silenced_{false};
  bool ground_enabled_{false};
  rclcpp::Time last_target_time_;

  // ROS 接口
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveController>());
  rclcpp::shutdown();
  return 0;
}
