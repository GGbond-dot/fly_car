#ifndef PID_CONTROL_PKG__PID_CONTROLLER_HPP_
#define PID_CONTROL_PKG__PID_CONTROLLER_HPP_

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pid_control_pkg
{

class PIDController
{
public:
  PIDController(
    double kp,
    double ki,
    double kd,
    double max_output = 1.0,
    double min_output = -1.0,
    double integral_limit = 10.0,
    double deadzone = 0.0);

  double calculate(double setpoint, double measured_value, double dt);
  void reset();
  void setPID(double kp, double ki, double kd);
  void setOutputLimits(double max_output, double min_output);
  void setIntegralLimit(double integral_limit);
  void setDeadzone(double deadzone);

  double getError() const { return current_error_; }
  double getIntegral() const { return integral_; }

private:
  double kp_;
  double ki_;
  double kd_;
  double max_output_;
  double min_output_;
  double integral_limit_;
  double deadzone_;
  double prev_error_;
  double current_error_;
  double integral_;
  double prev_derivative_;
  bool first_call_;
  double derivative_filter_alpha_;
};

enum class ControlMode
{
  NORMAL = 0,
  SLOW = 1,
  LOCK_Y = 2,
  LOCK_X = 3,
  HOVER = 4
};

class PositionPIDController : public rclcpp::Node
{
public:
  PositionPIDController();
  ~PositionPIDController() override = default;

private:
  void targetPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void controlTimerCallback();

  bool getCurrentPose();
  void loadParameters();
  void calculateErrors();
  double normalizeAngleDeg(double angle_deg) const;
  bool isTargetReached() const;
  void setControlMode(ControlMode mode);
  std_msgs::msg::Float32MultiArray processPID(double dt);

  inline double meterToCm(double meter) const { return meter * 100.0; }
  inline double radToDeg(double rad) const { return rad * 180.0 / M_PI; }

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // PID controllers
  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_yaw_;
  PIDController pid_z_;
  PIDController pid_xy_speed_;

  // Targets (cm, deg)
  double target_x_cm_;
  double target_y_cm_;
  double target_z_cm_;
  double target_yaw_deg_;
  bool has_target_position_;
  bool has_target_height_;

  // Current state
  double current_x_cm_;
  double current_y_cm_;
  double current_yaw_deg_;
  double current_z_cm_;
  bool has_current_pose_;

  // Control parameters
  double control_frequency_;
  std::string map_frame_;
  std::string laser_link_frame_;

  ControlMode control_mode_;
  double position_tolerance_;
  double yaw_tolerance_;
  double height_tolerance_;

  double max_linear_vel_;
  double max_angular_vel_;
  double max_vertical_vel_;
  double max_slow_vel_;

  double distance_xy_cm_;
  double error_x_cm_;
  double error_y_cm_;
  double error_yaw_deg_;
  double error_z_cm_;

  rclcpp::Time last_update_time_;
};

}  // namespace pid_control_pkg

#endif  // PID_CONTROL_PKG__PID_CONTROLLER_HPP_
