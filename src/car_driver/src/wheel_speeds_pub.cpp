#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class WheelSpeedsPublisher : public rclcpp::Node {
public:
  WheelSpeedsPublisher() : Node("wheel_speeds_pub") {
    this->declare_parameter("m1_speed",0.4);
    this->declare_parameter("m2_speed",0.4);
    this->declare_parameter("m3_speed", 0.4);
    this->declare_parameter("m4_speed", 0.4);

    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speeds", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&WheelSpeedsPublisher::timer_callback, this));
  }
  double target_x_cm_;
  double target_y_cm_;
  double target_z_cm_;
  double target_yaw_deg_;
  int has_target_position_ = false;

  double current_x_cm_;
  double current_y_cm_;
  double current_yaw_deg_;

private:
  void timer_callback() {
    auto msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
    double m1 = this->get_parameter("m1_speed").as_double();
    double m2 = this->get_parameter("m2_speed").as_double();
    double m3 = this->get_parameter("m3_speed").as_double();
    double m4 = this->get_parameter("m4_speed").as_double();
    msg->data = {static_cast<float>(m1), static_cast<float>(m2), static_cast<float>(m3), static_cast<float>(m4)};
    pub_->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "Published wheel speeds: %.2f, %.2f, %.2f, %.2f", m1, m2, m3, m4);
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // void PositionPIDController::targetPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  // {
  //   if (msg->data.size() < 4) {
  //     RCLCPP_WARN(get_logger(), "Target position message requires 4 floats [x_cm, y_cm, z_cm, yaw_deg]");
  //     return;
  //   }

  //   target_x_cm_ = static_cast<double>(msg->data[0]);
  //   target_y_cm_ = static_cast<double>(msg->data[1]);
  //   target_z_cm_ = static_cast<double>(msg->data[2]);
  //   target_yaw_deg_ = static_cast<double>(msg->data[3]);
  //   has_target_position_ = true;

  //   RCLCPP_INFO(get_logger(),
  //     "Received target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg",
  //     target_x_cm_, target_y_cm_, target_z_cm_, target_yaw_deg_);
  // }

  // bool PositionPIDController::getCurrentPose()
  // {
  //   try {
  //     // 查询从 "map" 坐标系到 "laser_link" 坐标系的变换
  //     geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
  //       map_frame_, laser_link_frame_, tf2::TimePointZero);

  //     // 从变换中提取X, Y坐标并转换为厘米
  //     current_x_cm_ = meterToCm(transform.transform.translation.x);
  //     current_y_cm_ = meterToCm(transform.transform.translation.y);

  //     // 从变换的四元数中计算出Yaw偏航角
  //     tf2::Quaternion q;
  //     tf2::fromMsg(transform.transform.rotation, q);
  //     double roll, pitch, yaw;
  //     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  //     current_yaw_deg_ = radToDeg(yaw); // 转换为角度

  //     has_current_pose_ = true;
  //     return true;
  //   } catch (const tf2::TransformException & ex) {
  //     RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
  //       "Failed to lookup transform %s -> %s: %s",
  //       map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
  //     return false;
  //   }
  // }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelSpeedsPublisher>());
  rclcpp::shutdown();
  return 0;
}