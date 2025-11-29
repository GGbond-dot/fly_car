// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "std_msgs/msg/float32_multi_array.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2/LinearMath/Matrix3x3.h"
// #include "pid_controller.hpp"
// // #include "amp_interfaces/msg/target_position.hpp" // 不再需要这个头文件
// #include <cmath>

// using rclcpp_lifecycle::LifecycleNode;
// using rclcpp_lifecycle::LifecyclePublisher;
// using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// class ControlNode : public LifecycleNode {
// public:
//     ControlNode() : LifecycleNode("control_node_lifecycle"),
//                     tf_buffer_(this->get_clock()),
//                     tf_listener_(tf_buffer_),
//                     wheel_radius_(0.04),
//                     wheel_base_(0.21),
//                     track_width_(0.20) {
//         // ... 参数声明部分保持不变 ...
//         this->declare_parameter<double>("pid_x_kp", 0.3);
//         this->declare_parameter<double>("pid_x_ki", 0.0);
//         this->declare_parameter<double>("pid_x_kd", 0.0);
//         this->declare_parameter<double>("pid_x_max_output", 0.4);
//         this->declare_parameter<double>("pid_x_dead_zone", 0.0);

//         this->declare_parameter<double>("pid_y_kp", 0.25);
//         this->declare_parameter<double>("pid_y_ki", 0.0);
//         this->declare_parameter<double>("pid_y_kd", 0.0);
//         this->declare_parameter<double>("pid_y_max_output", 0.4);
//         this->declare_parameter<double>("pid_y_dead_zone", 0.0);

//         this->declare_parameter<double>("pid_yaw_kp", 0.5);
//         this->declare_parameter<double>("pid_yaw_ki", 0.0);
//         this->declare_parameter<double>("pid_yaw_kd", 0.0);
//         this->declare_parameter<double>("pid_yaw_max_output", 0.8);
//         this->declare_parameter<double>("pid_yaw_dead_zone", 0.0);

//         // 初始化目标值为0
//         target_x_cm_ = 0.0;
//         target_y_cm_ = 0.0;
//         target_yaw_deg_ = 0.0;
//         is_active_ = false;
//     }

//     CallbackReturn on_configure(const rclcpp_lifecycle::State &)
//     {
//         update_pid_params();
//         // 步骤2: 修改订阅创建，使用 std_msgs::msg::Float32MultiArray
//         target_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
//             "target_position", 10, std::bind(&ControlNode::targetCallback, this, std::placeholders::_1));
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(10),
//             std::bind(&ControlNode::timerCallback, this));
//         velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("target_velocity", 10);
//         wheel_speeds_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speeds", 10);
//         return CallbackReturn::SUCCESS;
//     }

//     // ... on_activate, on_deactivate, on_cleanup, on_shutdown 保持不变 ...
//     // ...existing code...
//     CallbackReturn on_activate(const rclcpp_lifecycle::State &)
//     {
//         velocity_pub_->on_activate();
//         wheel_speeds_pub_->on_activate();
//         is_active_ = true;  // 添加活跃状态标志
//         RCLCPP_INFO(this->get_logger(), "ControlNode activated.");
//         return CallbackReturn::SUCCESS;
//     }

//     CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
//     {
//         is_active_ = false;  // 设置为非活跃状态
//         velocity_pub_->on_deactivate();
//         wheel_speeds_pub_->on_deactivate();
//         RCLCPP_INFO(this->get_logger(), "ControlNode deactivated.");
//         return CallbackReturn::SUCCESS;
//     }

//     CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
//     {
//         target_sub_.reset();
//         timer_.reset();
//         velocity_pub_.reset();
//         wheel_speeds_pub_.reset();
//         RCLCPP_INFO(this->get_logger(), "ControlNode cleaned up.");
//         return CallbackReturn::SUCCESS;
//     }

//     CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
//     {
//         target_sub_.reset();
//         timer_.reset();
//         velocity_pub_.reset();
//         wheel_speeds_pub_.reset();
//         RCLCPP_INFO(this->get_logger(), "ControlNode shutdown.");
//         return CallbackReturn::SUCCESS;
//     }
// private:
//     // ... update_pid_params 保持不变 ...
//     // ...existing code...
//     void update_pid_params() {
//         double x_kp = this->get_parameter("pid_x_kp").as_double();
//         double x_ki = this->get_parameter("pid_x_ki").as_double();
//         double x_kd = this->get_parameter("pid_x_kd").as_double();
//         double x_max_output = this->get_parameter("pid_x_max_output").as_double();
//         double x_dead_zone = this->get_parameter("pid_x_dead_zone").as_double();

//         double y_kp = this->get_parameter("pid_y_kp").as_double();
//         double y_ki = this->get_parameter("pid_y_ki").as_double();
//         double y_kd = this->get_parameter("pid_y_kd").as_double();
//         double y_max_output = this->get_parameter("pid_y_max_output").as_double();
//         double y_dead_zone = this->get_parameter("pid_y_dead_zone").as_double();

//         double yaw_kp = this->get_parameter("pid_yaw_kp").as_double();
//         double yaw_ki = this->get_parameter("pid_yaw_ki").as_double();
//         double yaw_kd = this->get_parameter("pid_yaw_kd").as_double();
//         double yaw_max_output = this->get_parameter("pid_yaw_max_output").as_double();
//         double yaw_dead_zone = this->get_parameter("pid_yaw_dead_zone").as_double();

//         pid_x_.set_params(x_kp, x_ki, x_kd, x_max_output, x_dead_zone);
//         pid_y_.set_params(y_kp, y_ki, y_kd, y_max_output, y_dead_zone);
//         pid_yaw_.set_params(yaw_kp, yaw_ki, yaw_kd, yaw_max_output, yaw_dead_zone);

//         RCLCPP_INFO(this->get_logger(), "PID参数已更新: x[%.3f, %.3f, %.3f, max=%.2f, dead=%.3f] y[%.3f, %.3f, %.3f, max=%.2f, dead=%.3f] yaw[%.3f, %.3f, %.3f, max=%.2f, dead=%.3f]",
//             x_kp, x_ki, x_kd, x_max_output, x_dead_zone,
//             y_kp, y_ki, y_kd, y_max_output, y_dead_zone,
//             yaw_kp, yaw_ki, yaw_kd, yaw_max_output, yaw_dead_zone);
//     }

//     // 步骤3: 修改回调函数以处理 Float32MultiArray
//     void targetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
//         if (msg->data.size() < 4) {
//             RCLCPP_WARN(this->get_logger(), "Received target position message with insufficient data. Expected 4 floats.");
//             return;
//         }
        
//         // 从数组中解析数据
//         target_x_cm_ = static_cast<double>(msg->data[0]);
//         target_y_cm_ = static_cast<double>(msg->data[1]);
//         // msg->data[2] 是 z_cm，在这个2D PID控制器中我们忽略它
//         target_yaw_deg_ = static_cast<double>(msg->data[3]);

//         RCLCPP_INFO(this->get_logger(), "Received target position: x=%.2f cm, y=%.2f cm, yaw=%.2f deg",
//                     target_x_cm_, target_y_cm_, target_yaw_deg_);
//     }

//     void timerCallback() {
//         if (!is_active_) {
//             return;
//         }
        
//         try {
//             geometry_msgs::msg::TransformStamped transform_stamped = 
//                 tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
            
//             // 注意单位转换：TF提供米，而目标是厘米
//             double current_x_m = transform_stamped.transform.translation.x;
//             double current_y_m = transform_stamped.transform.translation.y;
            
//             tf2::Quaternion q(
//                 transform_stamped.transform.rotation.x,
//                 transform_stamped.transform.rotation.y,
//                 transform_stamped.transform.rotation.z,
//                 transform_stamped.transform.rotation.w);
//             tf2::Matrix3x3 m(q);
//             double roll, pitch, yaw_rad;
//             m.getRPY(roll, pitch, yaw_rad);
            
//             // 1. 计算全局坐标系下的位置误差 (单位：米)
//             double error_x_global = (target_x_cm_ / 100.0) - current_x_m;
//             double error_y_global = (target_y_cm_ / 100.0) - current_y_m;
            
//             // 2. 计算角度误差 (单位：弧度)
//             double target_yaw_rad = target_yaw_deg_ * M_PI / 180.0;
//             double error_yaw = normalizeAngle(target_yaw_rad - yaw_rad);
            
//             // ... 后续的PID计算和发布逻辑保持不变 ...
//             // ...existing code...
//             // 3. 将全局坐标系下的位置误差转换到机器人本体坐标系
//             double error_x_local, error_y_local;
//             globalToLocal(error_x_global, error_y_global, yaw_rad, error_x_local, error_y_local);
            
//             // 4. PID控制器计算本体坐标系下的速度指令
//             double vx_local = pid_x_.compute(error_x_local, 0.0, 0.01);
//             double vy_local = pid_y_.compute(error_y_local, 0.0, 0.01);
//             double angular_velocity = pid_yaw_.compute(error_yaw, 0.0, 0.01);

//             // 5. 发布Twist消息（这里发布的是本体坐标系下的速度）
//             auto velocity_msg = geometry_msgs::msg::Twist();
//             velocity_msg.linear.x = vx_local;
//             velocity_msg.linear.y = vy_local;
//             velocity_msg.angular.z = angular_velocity;
//             velocity_pub_->publish(velocity_msg);
            
//             // 6. 计算麦克纳姆轮速度（使用本体坐标系下的速度）
//             calculateMecanumWheelSpeeds(vx_local, vy_local, angular_velocity);
            
//             // 调试信息
//             if (std::abs(error_x_global) > 0.01 || std::abs(error_y_global) > 0.01 || std::abs(error_yaw) > 0.01) {
//                 RCLCPP_DEBUG(this->get_logger(), 
//                     "Global error: x=%.3f, y=%.3f | Local error: x=%.3f, y=%.3f | Yaw error: %.3f | Current yaw: %.3f",
//                     error_x_global, error_y_global, error_x_local, error_y_local, error_yaw, yaw_rad);
//                 RCLCPP_DEBUG(this->get_logger(),
//                     "Velocities: vx_local=%.3f, vy_local=%.3f, omega=%.3f",
//                     vx_local, vy_local, angular_velocity);
//             }
            
//         } catch (tf2::TransformException &ex) {
//             RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
//         }
//     }

//     // ... globalToLocal, localToGlobal, normalizeAngle, calculateMecanumWheelSpeeds 保持不变 ...
//     // ...existing code...
//     void globalToLocal(double x_global, double y_global, double robot_yaw, 
//                        double& x_local, double& y_local) {
//         // 旋转变换矩阵：从全局坐标系到本体坐标系
//         // [x_local]   [cos(θ)  sin(θ)] [x_global]
//         // [y_local] = [-sin(θ) cos(θ)] [y_local]
//         double cos_yaw = std::cos(robot_yaw);
//         double sin_yaw = std::sin(robot_yaw);
        
//         x_local = cos_yaw * x_global + sin_yaw * y_global;
//         y_local = -sin_yaw * x_global + cos_yaw * y_global;
//     }

//     /**
//      * 将机器人本体坐标系下的坐标转换到全局坐标系（备用函数）
//      */
//     void localToGlobal(double x_local, double y_local, double robot_yaw,
//                        double& x_global, double& y_global) {
//         // 旋转变换矩阵：从本体坐标系到全局坐标系
//         // [x_global]   [cos(θ) -sin(θ)] [x_local]
//         // [y_global] = [sin(θ)  cos(θ)] [y_local]
//         double cos_yaw = std::cos(robot_yaw);
//         double sin_yaw = std::sin(robot_yaw);
        
//         x_global = cos_yaw * x_local - sin_yaw * y_local;
//         y_global = sin_yaw * x_local + cos_yaw * y_local;
//     }

//     double normalizeAngle(double angle) {
//         while (angle > M_PI) angle -= 2.0 * M_PI;
//         while (angle < -M_PI) angle += 2.0 * M_PI;
//         return angle;
//     }

//     void calculateMecanumWheelSpeeds(double vx, double vy, double omega) {
//         // 麦克纳姆轮运动学公式
//         // vx: 机器人本体坐标系下前进方向速度
//         // vy: 机器人本体坐标系下左侧方向速度  
//         // omega: 角速度（逆时针为正）
        
//         double wheel_speeds[4];
//         double lx = wheel_base_ / 2.0;  // 前后轮距的一半
//         double ly = track_width_ / 2.0; // 左右轮距的一半
        
//         // 麦克纳姆轮速度计算公式（标准配置）
//         // 前左轮 (Front Left)
//         wheel_speeds[0] = (vx - vy - omega * (lx + ly));
//         // 前右轮 (Front Right)  
//         wheel_speeds[1] = (vx + vy - omega * (lx + ly));
//         // 后右轮 (Rear Right)
//         wheel_speeds[2] = (vx + vy + omega * (lx + ly));
//         // 后左轮 (Rear Left)
//         wheel_speeds[3] = (vx - vy + omega * (lx + ly));
        
//         auto wheel_msg = std_msgs::msg::Float32MultiArray();
//         wheel_msg.data.resize(4);
//         for (int i = 0; i < 4; i++) {
//             wheel_msg.data[i] = wheel_speeds[i];
//         }
//         wheel_speeds_pub_->publish(wheel_msg);

//         // 详细调试信息（可选开启）
//         // RCLCPP_INFO(this->get_logger(), 
//         //     "Local velocities: vx=%.3f, vy=%.3f, omega=%.3f", vx, vy, omega);
//         // RCLCPP_INFO(this->get_logger(), 
//         //     "Wheel speeds - FL: %.3f, FR: %.3f, RR: %.3f, RL: %.3f", 
//         //     wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3]);
//     }

//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;
//     PIDController pid_x_{0.3, 0.0, 0.0, 1.0, 0.0};
//     PIDController pid_y_{0.25, 0.0, 0.0, 1.0, 0.0};
//     PIDController pid_yaw_{0.5, 0.0, 0.0, 0.8, 0.0};
//     double wheel_radius_;
//     double wheel_base_;
//     double track_width_;
    
//     // 步骤1: 修改成员变量类型
//     rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;
//     std::shared_ptr<LifecyclePublisher<geometry_msgs::msg::Twist>> velocity_pub_;
//     std::shared_ptr<LifecyclePublisher<std_msgs::msg::Float32MultiArray>> wheel_speeds_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
    
//     // 使用独立的 double 变量存储目标值
//     double target_x_cm_;
//     double target_y_cm_;
//     double target_yaw_deg_;

//     bool is_active_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ControlNode>();
//     rclcpp::spin(node->get_node_base_interface());
//     rclcpp::shutdown();
//     return 0;
// }




#include "pid_control_pkg/pid_controller.hpp"

#include <angles/angles.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pid_control_pkg
{

PIDController::PIDController(
  double kp,
  double ki,
  double kd,
  double max_output,
  double min_output,
  double integral_limit,
  double deadzone)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  max_output_(max_output),
  min_output_(min_output),
  integral_limit_(integral_limit),
  deadzone_(deadzone),
  prev_error_(0.0),
  current_error_(0.0),
  integral_(0.0),
  prev_derivative_(0.0),
  first_call_(true),
  derivative_filter_alpha_(0.8)
{
}

double PIDController::calculate(double setpoint, double measured_value, double dt)
{
  current_error_ = setpoint - measured_value;

  if (std::fabs(current_error_) < deadzone_) {
    current_error_ = 0.0;
  }

  if (first_call_) {
    prev_error_ = current_error_;
    first_call_ = false;
  }

  const double proportional = kp_ * current_error_;

  integral_ += current_error_ * dt;
  if (integral_ > integral_limit_) {
    integral_ = integral_limit_;
  } else if (integral_ < -integral_limit_) {
    integral_ = -integral_limit_;
  }
  const double integral_term = ki_ * integral_;

  const double derivative_raw = (dt > 0.0) ? (current_error_ - prev_error_) / dt : 0.0;
  const double derivative_filtered = derivative_filter_alpha_ * prev_derivative_ +
    (1.0 - derivative_filter_alpha_) * derivative_raw;
  const double derivative_term = kd_ * derivative_filtered;

  double output = proportional + integral_term + derivative_term;

  if (output > max_output_) {
    output = max_output_;
  } else if (output < min_output_) {
    output = min_output_;
  }

  if ((output >= max_output_ && current_error_ > 0.0) ||
    (output <= min_output_ && current_error_ < 0.0))
  {
    integral_ -= current_error_ * dt;
  }

  prev_error_ = current_error_;
  prev_derivative_ = derivative_filtered;

  return output;
}

void PIDController::reset()
{
  prev_error_ = 0.0;
  current_error_ = 0.0;
  integral_ = 0.0;
  prev_derivative_ = 0.0;
  first_call_ = true;
}

void PIDController::setPID(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setOutputLimits(double max_output, double min_output)
{
  max_output_ = max_output;
  min_output_ = min_output;
}

void PIDController::setIntegralLimit(double integral_limit)
{
  integral_limit_ = integral_limit;
}

void PIDController::setDeadzone(double deadzone)
{
  deadzone_ = deadzone;
}
/*
  订阅/发布主题：
    订阅 /target_position (std_msgs::msg::Float32MultiArray)：接收目标位置和朝向，包含四个浮点数 [x_cm, y_cm, z_cm, yaw_deg]。
    订阅 /height 
    发布 /target_velocity (std_msgs::msg::Float32MultiArray)
*/
PositionPIDController::PositionPIDController()
: rclcpp::Node("position_pid_controller"),
  pid_x_(0.5, 0.0, 0.1, 36.0, -33.0, 5.0, 0.6),
  pid_y_(0.5, 0.0, 0.1, 36.0, -33.0, 5.0, 0.6),
  pid_yaw_(1.0, 0.0, 0.2, 30.0, -30.0, 2.0, 0.5),
  pid_z_(0.9, 0.0, 0.15, 35.0, -60.0, 3.0, 0.6),
  pid_xy_speed_(0.5, 0.0, 0.1, 36.0, -36.0, 5.0, 0.6),
  target_x_cm_(0.0),
  target_y_cm_(0.0),
  target_z_cm_(0.0),
  target_yaw_deg_(0.0),
  has_target_position_(false),
  has_target_height_(false),
  current_x_cm_(0.0),
  current_y_cm_(0.0),
  current_yaw_deg_(0.0),
  current_z_cm_(0.0),
  has_current_pose_(false),
  control_frequency_(50.0),
  map_frame_("map"),
  laser_link_frame_("laser_link"),
  control_mode_(ControlMode::NORMAL),
  position_tolerance_(6.0),
  yaw_tolerance_(5.0),
  height_tolerance_(6.0),
  max_linear_vel_(36.0),
  max_angular_vel_(30.0),
  max_vertical_vel_(40.0),
  max_slow_vel_(20.0),
  distance_xy_cm_(0.0),
  error_x_cm_(0.0),
  error_y_cm_(0.0),
  error_yaw_deg_(0.0),
  error_z_cm_(0.0),
  last_update_time_(now())
{
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  target_position_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/target_position", rclcpp::QoS(10),
    std::bind(&PositionPIDController::targetPositionCallback, this, std::placeholders::_1));

    // target_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    //         "target_position", 10, std::bind(&ControlNode::targetCallback, this, std::placeholders::_1));

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&PositionPIDController::heightCallback, this, std::placeholders::_1));

  target_velocity_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/target_velocity", rclcpp::QoS(10));

  const double period_sec = 1.0 / std::max(control_frequency_, 1.0);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec)),
    std::bind(&PositionPIDController::controlTimerCallback, this));

  RCLCPP_INFO(get_logger(), "Position PID Controller initialized (%.1f Hz)", control_frequency_);
  RCLCPP_INFO(get_logger(), "Frames: map=%s, laser_link=%s", map_frame_.c_str(), laser_link_frame_.c_str());
}

void PositionPIDController::targetPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(get_logger(), "Target position message requires 4 floats [x_cm, y_cm, z_cm, yaw_deg]");
    return;
  }

  target_x_cm_ = static_cast<double>(msg->data[0]);
  target_y_cm_ = static_cast<double>(msg->data[1]);
  target_z_cm_ = static_cast<double>(msg->data[2]);
  target_yaw_deg_ = static_cast<double>(msg->data[3]);
  has_target_position_ = true;

  RCLCPP_INFO(get_logger(),
    "Received target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",
    target_x_cm_, target_y_cm_, target_z_cm_, target_yaw_deg_);
}

void PositionPIDController::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_z_cm_ = static_cast<double>(msg->data);
  has_target_height_ = true;
}
/*
    获取自动获取tf数据
*/
bool PositionPIDController::getCurrentPose()
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);

    current_x_cm_ = meterToCm(transform.transform.translation.x);
    current_y_cm_ = meterToCm(transform.transform.translation.y);

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_deg_ = radToDeg(yaw);

    has_current_pose_ = true;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Failed to lookup transform %s -> %s: %s",
      map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

double PositionPIDController::normalizeAngleDeg(double angle_deg) const
{
  double result = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(result);
}

void PositionPIDController::calculateErrors()
{
  error_x_cm_ = target_x_cm_ - current_x_cm_;
  error_y_cm_ = target_y_cm_ - current_y_cm_;
  distance_xy_cm_ = std::hypot(error_x_cm_, error_y_cm_);
  error_yaw_deg_ = normalizeAngleDeg(target_yaw_deg_ - current_yaw_deg_);
  if (has_target_height_) {
    error_z_cm_ = target_z_cm_ - current_z_cm_;
  } else {
    error_z_cm_ = 0.0;
  }
}

bool PositionPIDController::isTargetReached() const
{
  return std::fabs(error_x_cm_) <= position_tolerance_ &&
         std::fabs(error_y_cm_) <= position_tolerance_ &&
         std::fabs(error_yaw_deg_) <= yaw_tolerance_ &&
         (!has_target_height_ || std::fabs(error_z_cm_) <= height_tolerance_);
}

void PositionPIDController::setControlMode(ControlMode mode)
{
  control_mode_ = mode;
  double limit = max_linear_vel_;

  switch (mode) {
    case ControlMode::NORMAL:
      limit = max_linear_vel_;
      break;
    case ControlMode::SLOW:
    case ControlMode::LOCK_Y:
    case ControlMode::LOCK_X:
      limit = max_slow_vel_;
      break;
    case ControlMode::HOVER:
      limit = 10.0;
      break;
  }

  pid_x_.setOutputLimits(limit, -limit);
  pid_y_.setOutputLimits(limit, -limit);
  pid_xy_speed_.setOutputLimits(limit, -limit);
}

std_msgs::msg::Float32MultiArray PositionPIDController::processPID(double dt)
{
  std_msgs::msg::Float32MultiArray cmd;
  cmd.data.resize(4);

  calculateErrors();

  double vel_x_cm = 0.0;
  double vel_y_cm = 0.0;

  switch (control_mode_) {
    case ControlMode::NORMAL:
    case ControlMode::SLOW:
    {
      if (distance_xy_cm_ > 0.1) {
        double speed_cmd = -pid_xy_speed_.calculate(0.0, distance_xy_cm_, dt);
        if (speed_cmd < 0.0) {
          speed_cmd = 0.0;
        }
        const double cos_theta = error_x_cm_ / distance_xy_cm_;
        const double sin_theta = error_y_cm_ / distance_xy_cm_;
        vel_x_cm = speed_cmd * cos_theta;
        vel_y_cm = speed_cmd * sin_theta;
      } else {
        vel_x_cm = 0.0;
        vel_y_cm = 0.0;
      }
      break;
    }
    case ControlMode::LOCK_Y:
      vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
      vel_x_cm = 0.4 * pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
      break;
    case ControlMode::LOCK_X:
      vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
      vel_y_cm = 0.4 * pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
      break;
    case ControlMode::HOVER:
      vel_x_cm = pid_x_.calculate(target_x_cm_, current_x_cm_, dt);
      vel_y_cm = pid_y_.calculate(target_y_cm_, current_y_cm_, dt);
      break;
  }

  const double vel_yaw_deg = pid_yaw_.calculate(0.0, -error_yaw_deg_, dt);
  const double vel_z_cm = has_target_height_ ?
    pid_z_.calculate(target_z_cm_, current_z_cm_, dt) : 0.0;

  // double vel_z_cm = 0.0;
  // if(has_target_height_)
  // {
  //   double pid_z_output = pid_z_.calculate(target_z_cm_, current_z_cm_, dt);
  //   double err_z = target_z_cm_ - current_z_cm_;
  //   double abs_err_z = std::fabs(err_z);

  //   const double min_err = 2.0; // cm
  //   const double medium_err = 10.0; // cm
  //   const double min_boost = 10.0; // cm/s

  //   if(abs_err_z < min_err)
  //   {
  //     // 误差很小，直接悬停
  //     vel_z_cm = pid_z_output;
  //   }
  //   else if(abs_err_z < medium_err)
  //   {
  //     // 误差中等，减速
  //     vel_z_cm = pid_z_output + min_boost*(err_z > 0 ? 1 : -1);
  //   }
  //   else
  //   {
  //     // 误差较大，正常速度
  //     vel_z_cm = pid_z_output;
  //   }
  //}

  cmd.data[0] = static_cast<float>(vel_x_cm);
  cmd.data[1] = static_cast<float>(vel_y_cm);
  cmd.data[2] = static_cast<float>(vel_z_cm);
  cmd.data[3] = static_cast<float>(vel_yaw_deg);
  return cmd;
}

void PositionPIDController::controlTimerCallback()
{
  if (!has_target_position_) {
    return;
  }

  if (!getCurrentPose()) {
    return;
  }

  const rclcpp::Time now_time = now();
  double dt = (now_time - last_update_time_).seconds();
  if (dt <= 0.0) {
    dt = 1.0 / std::max(control_frequency_, 1.0);
  }
  last_update_time_ = now_time;

  auto cmd_vel = processPID(dt);
  target_velocity_pub_->publish(cmd_vel);

  if (isTargetReached()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Target reached: distance=%.1fcm yaw_error=%.1fdeg",
      distance_xy_cm_, error_yaw_deg_);
  }

  RCLCPP_DEBUG(get_logger(),
    "Current[%.1f, %.1f, %.1fdeg] Target[%.1f, %.1f, %.1fdeg] Error[%.1f, %.1f, %.1fdeg]",
    current_x_cm_, current_y_cm_, current_yaw_deg_,
    target_x_cm_, target_y_cm_, target_yaw_deg_,
    error_x_cm_, error_y_cm_, error_yaw_deg_);
}

void PositionPIDController::loadParameters()
{
  control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  laser_link_frame_ = declare_parameter<std::string>("laser_link_frame", "laser_link");
  position_tolerance_ = declare_parameter<double>("position_tolerance", 6.0);
  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 5.0);
  height_tolerance_ = declare_parameter<double>("height_tolerance", 6.0);

  const double kp_xy = declare_parameter<double>("kp_xy", 0.5);
  const double ki_xy = declare_parameter<double>("ki_xy", 0.0);
  const double kd_xy = declare_parameter<double>("kd_xy", 0.1);

  const double kp_yaw = declare_parameter<double>("kp_yaw", 1.0);
  const double ki_yaw = declare_parameter<double>("ki_yaw", 0.0);
  const double kd_yaw = declare_parameter<double>("kd_yaw", 0.2);

  const double kp_z = declare_parameter<double>("kp_z", 0.9);
  const double ki_z = declare_parameter<double>("ki_z", 0.0);
  const double kd_z = declare_parameter<double>("kd_z", 0.15);

  max_linear_vel_ = declare_parameter<double>("max_linear_velocity", 36.0);
  max_angular_vel_ = declare_parameter<double>("max_angular_velocity", 30.0);
  max_vertical_vel_ = declare_parameter<double>("max_vertical_velocity", 40.0);
  max_slow_vel_ = declare_parameter<double>("max_slow_velocity", 20.0);

  pid_x_.setPID(kp_xy, ki_xy, kd_xy);
  pid_y_.setPID(kp_xy, ki_xy, kd_xy);
  pid_yaw_.setPID(kp_yaw, ki_yaw, kd_yaw);
  pid_z_.setPID(kp_z, ki_z, kd_z);
  pid_xy_speed_.setPID(kp_xy, ki_xy, kd_xy);

  pid_x_.setOutputLimits(max_linear_vel_, -max_linear_vel_);
  pid_y_.setOutputLimits(max_linear_vel_, -max_linear_vel_);
  pid_yaw_.setOutputLimits(max_angular_vel_, -max_angular_vel_);
  pid_z_.setOutputLimits(max_vertical_vel_, -60.0);
  pid_xy_speed_.setOutputLimits(max_linear_vel_, -max_linear_vel_);

  RCLCPP_INFO(get_logger(),
    "PID params: XY(kp=%.2f, ki=%.2f, kd=%.2f) Yaw(kp=%.2f, ki=%.2f, kd=%.2f) Z(kp=%.2f, ki=%.2f, kd=%.2f)",
    kp_xy, ki_xy, kd_xy, kp_yaw, ki_yaw, kd_yaw, kp_z, ki_z, kd_z);
  RCLCPP_INFO(get_logger(),
    "Tolerances: pos=%.1fcm yaw=%.1fdeg height=%.1fcm",
    position_tolerance_, yaw_tolerance_, height_tolerance_);
  RCLCPP_INFO(get_logger(),
    "Velocity limits: linear=%.1fcm/s angular=%.1fdeg/s vertical=%.1fcm/s slow=%.1fcm/s",
    max_linear_vel_, max_angular_vel_, max_vertical_vel_, max_slow_vel_);
}

}  // namespace pid_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pid_control_pkg::PositionPIDController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
