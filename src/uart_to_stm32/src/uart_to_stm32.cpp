#include "uart_to_stm32/uart_to_stm32.hpp"
#include <iostream>
  
#include <chrono>
#include <cmath>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace uart_to_stm32
{

using namespace std::chrono_literals;

UartToStm32::UartToStm32(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  update_rate_(0.0),
  current_yaw_(0.0),
  yaw_valid_(false),
  velocity_valid_(false),
  has_st_ready_pub_(false)
{
  RCLCPP_INFO(node_->get_logger(), "UartToStm32 created");
}

UartToStm32::~UartToStm32()
{
  if (timer_) {
    timer_->cancel();
  }
  if (serial_comm_) {
    serial_comm_->stop_protocol_receive();
    serial_comm_->close();
  }
}

bool UartToStm32::initialize(double update_rate, const std::string & source_frame, const std::string & target_frame)
{
  try {
    update_rate_ = update_rate;
    source_frame_ = source_frame;
    target_frame_ = target_frame;

    RCLCPP_INFO(node_->get_logger(), "UartToStm32 initialized with update rate: %.1f Hz", update_rate_);
    RCLCPP_INFO(node_->get_logger(), "Looking for transform from '%s' to '%s'", source_frame_.c_str(), target_frame_.c_str());

    serial_comm_ = std::make_unique<serial_comm::SerialComm>();
    if (!serial_comm_->initialize("/dev/ttyS6", 921600)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize serial port /dev/ttyS6 at 921600 baudrate");
      RCLCPP_ERROR(node_->get_logger(), "Serial error: %s", serial_comm_->get_last_error().c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Serial port /dev/ttyS6 initialized at 921600 baudrate");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = node_->create_wall_timer(
      period,
      std::bind(&UartToStm32::lookupTransform, this));

    velocity_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/velocity_map", 10,
      std::bind(&UartToStm32::velocityCallback, this, std::placeholders::_1));

    target_velocity_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_velocity", 10,
      std::bind(&UartToStm32::targetVelocityCallback, this, std::placeholders::_1));

    laser_ground_height_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
      "/laser_array/ground_height", rclcpp::QoS(10),
      std::bind(&UartToStm32::laserGroundHeightCallback, this, std::placeholders::_1));

    height_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/height", 10);
    height_raw_stm32_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/height_raw_stm32", 10);
    is_st_ready_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/is_st_ready", rclcpp::QoS(10).transient_local());
    mission_step_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/mission_step", 10);

    has_st_ready_pub_ = false;

    // 标志位:高度源切换(默认 false=STM32 单点激光)。ros2 param set 可热切换。
    use_laser_array_height_ = node_->declare_parameter<bool>("use_laser_array_height", false);
    param_cb_handle_ = node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "use_laser_array_height") {
            use_laser_array_height_ = p.as_bool();
            RCLCPP_WARN(node_->get_logger(), "高度源切换 -> %s",
              use_laser_array_height_ ? "面阵激光(替换+回传STM32)" : "STM32单点激光(原样)");
          }
        }
        return result;
      });
    RCLCPP_INFO(node_->get_logger(), "高度源初始 = %s",
      use_laser_array_height_ ? "面阵激光" : "STM32单点激光");

    serial_comm_->start_protocol_receive(
      [this](uint8_t id, const std::vector<uint8_t> & data) { protocolDataHandler(id, data); },
      [this](const std::string & err) {
        RCLCPP_WARN(node_->get_logger(), "Serial protocol error: %s", err.c_str());
      });

    RCLCPP_INFO(node_->get_logger(), "UartToStm32 initialized successfully");
    RCLCPP_INFO(node_->get_logger(), "Subscribed to /velocity_map and /target_velocity topics");
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize topic subscriber: %s", e.what());
    return false;
  }
}

void UartToStm32::lookupTransform()
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      source_frame_, target_frame_, tf2::TimePointZero);
    processTfTransform(transform);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "Transform lookup failed: %s", ex.what());
  }
}

void UartToStm32::processTfTransform(const geometry_msgs::msg::TransformStamped & transform)
{
  const double x = transform.transform.translation.x;
  const double y = transform.transform.translation.y;
  const double z = transform.transform.translation.z;

  const double qx = transform.transform.rotation.x;
  const double qy = transform.transform.rotation.y;
  const double qz = transform.transform.rotation.z;
  const double qw = transform.transform.rotation.w;

  tf2::Quaternion q(qx, qy, qz, qw);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_yaw_ = yaw;
  yaw_valid_ = true;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Transform %s -> %s: pos(%.3f, %.3f, %.3f) rot(%.3f, %.3f, %.3f)",
    source_frame_.c_str(), target_frame_.c_str(), x, y, z, roll, pitch, yaw);

  if (velocity_valid_ && yaw_valid_) {
    Eigen::Vector3d linear_vel(
      current_velocity_.linear.x,
      current_velocity_.linear.y,
      current_velocity_.linear.z);
    const Eigen::Vector3d transformed_vel = transformVelocity(linear_vel, current_yaw_);
    sendVelocityToSerial(transformed_vel);
  }
}

void UartToStm32::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_velocity_ = *msg;
  velocity_valid_ = true;

  const double linear_x = msg->linear.x;
  const double linear_y = msg->linear.y;
  const double linear_z = msg->linear.z;
  const double angular_x = msg->angular.x;
  const double angular_y = msg->angular.y;
  const double angular_z = msg->angular.z;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Velocity: linear(%.3f, %.3f, %.3f) angular(%.3f, %.3f, %.3f)",
    linear_x, linear_y, linear_z, angular_x, angular_y, angular_z);

  if (yaw_valid_ && velocity_valid_) {
    Eigen::Vector3d linear_vel(linear_x, linear_y, linear_z);
    const Eigen::Vector3d transformed_vel = transformVelocity(linear_vel, current_yaw_);
    sendVelocityToSerial(transformed_vel);
  }
}


Eigen::Vector3d UartToStm32::transformVelocity(const Eigen::Vector3d & linear, double yaw)
{
  Eigen::Matrix3d Rz;
  Rz << std::cos(yaw), std::sin(yaw), 0.0,
    -std::sin(yaw), std::cos(yaw), 0.0,
    0.0, 0.0, 1.0;

  const Eigen::Vector3d transformed = Rz * linear;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Velocity transform: yaw=%.3f deg, original(%.3f,%.3f,%.3f) -> transformed(%.3f,%.3f,%.3f)",
    yaw * 180.0 / M_PI,
    linear.x(), linear.y(), linear.z(),
    transformed.x(), transformed.y(), transformed.z());

  return transformed;
}

void UartToStm32::sendVelocityToSerial(const Eigen::Vector3d & transformed_velocity)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send velocity data");
    return;
  }

  try {
    constexpr double scale_factor = 100.0;

    const int16_t vel_x = static_cast<int16_t>(transformed_velocity.x() * scale_factor);
    const int16_t vel_y = static_cast<int16_t>(transformed_velocity.y() * scale_factor);
    const int16_t vel_z = static_cast<int16_t>(transformed_velocity.z() * scale_factor);

    std::vector<uint8_t> data(6);
    data[0] = static_cast<uint8_t>(vel_x & 0xFF);
    data[1] = static_cast<uint8_t>((vel_x >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(vel_y & 0xFF);
    data[3] = static_cast<uint8_t>((vel_y >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(vel_z & 0xFF);
    data[5] = static_cast<uint8_t>((vel_z >> 8) & 0xFF);

    if (serial_comm_->send_protocol_data(VELOCITY_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "Sent velocity data: x=%d, y=%d, z=%d (cm/s)", vel_x, vel_y, vel_z);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Failed to send velocity data: %s", serial_comm_->get_last_error().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in sendVelocityToSerial: %s", e.what());
  }
}

void UartToStm32::targetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(node_->get_logger(),
      "Target velocity message should contain 4 float values [vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]");
    return;
  }

  const float vx_cm_per_s = msg->data[0];
  const float vy_cm_per_s = msg->data[1];
  const float vz_cm_per_s = msg->data[2];
  const float vyaw_deg_per_s = msg->data[3];

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Target Velocity: linear(%.1f, %.1f, %.1f)cm/s angular(%.1f)deg/s",
    vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);

  // 目标速度由 PID 在 map 系下计算得到，飞控期望机体系，
  // 因此与当前速度通路一致，用当前 yaw 做 map->body 旋转后再下发。
  if (yaw_valid_) {
    const Eigen::Vector3d v_map(vx_cm_per_s, vy_cm_per_s, vz_cm_per_s);
    const Eigen::Vector3d v_body = transformVelocity(v_map, current_yaw_);
    sendTargetVelocityToSerial(
      static_cast<float>(v_body.x()),
      static_cast<float>(v_body.y()),
      static_cast<float>(v_body.z()),
      vyaw_deg_per_s);
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "Yaw not yet valid, sending target velocity without map->body rotation");
    sendTargetVelocityToSerial(vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);
  }
}

void UartToStm32::sendTargetVelocityToSerial(float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send target velocity data");
    return;
  }

  try {
    const int16_t vel_x = static_cast<int16_t>(std::lround(vx_cm_per_s));
    const int16_t vel_y = static_cast<int16_t>(std::lround(vy_cm_per_s));
    const int16_t vel_z = static_cast<int16_t>(std::lround(vz_cm_per_s));
    const int16_t vel_yaw = static_cast<int16_t>(std::lround(vyaw_deg_per_s));

    std::vector<uint8_t> data(8);
    data[0] = static_cast<uint8_t>(vel_x & 0xFF);
    data[1] = static_cast<uint8_t>((vel_x >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(vel_y & 0xFF);
    data[3] = static_cast<uint8_t>((vel_y >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(vel_z & 0xFF);
    data[5] = static_cast<uint8_t>((vel_z >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>(vel_yaw & 0xFF);
    data[7] = static_cast<uint8_t>((vel_yaw >> 8) & 0xFF);

    if (serial_comm_->send_protocol_data(TARGET_VELOCITY_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "Sent target velocity data: x=%d, y=%d, z=%d, yaw=%d",
        vel_x, vel_y, vel_z, vel_yaw);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Failed to send target velocity data: %s", serial_comm_->get_last_error().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in sendTargetVelocityToSerial: %s", e.what());
  }
}

void UartToStm32::laserGroundHeightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  // 标志位=STM32 时,面阵激光既不占据 /height 也不回传,完全旁路。
  if (!use_laser_array_height_) {
    return;
  }
  // 替换:面阵高度成为飞控 PID/mux 用的 /height
  if (height_pub_) {
    height_pub_->publish(*msg);
  }
  // 回传:把面阵高度用 0x07 帧下发 STM32,供飞控内部定高
  sendLaserGroundHeightToSerial(msg->data);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "Published /height(面阵) + 回传STM32: %d", static_cast<int>(msg->data));
}

void UartToStm32::sendLaserGroundHeightToSerial(int16_t height_cm)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send laser ground height");
    return;
  }
  std::vector<uint8_t> data(2);
  data[0] = static_cast<uint8_t>(height_cm & 0xFF);
  data[1] = static_cast<uint8_t>((height_cm >> 8) & 0xFF);
  if (!serial_comm_->send_protocol_data(
        LASER_GROUND_HEIGHT_FRAME_ID, static_cast<uint8_t>(data.size()), data)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Failed to send laser ground height: %s", serial_comm_->get_last_error().c_str());
  }
}

void UartToStm32::protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data)
{
  switch (id) {
    case ST_READY_QUERY_ID: {
      if (data.size() < 9) {
        RCLCPP_WARN(node_->get_logger(), "protocolDataHandler: ID 0xF1 data too short, len=%zu", data.size());
        break;
      }
      const uint8_t first = data[0];
      if (mission_step_pub_) {
        std_msgs::msg::UInt8 msg;
        msg.data = first;
        mission_step_pub_->publish(msg);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "Published /mission_step: %u (from 0xF1 frame)", static_cast<unsigned>(first));
      }
      if (has_st_ready_pub_) {
        break;
      }
      const uint8_t second = data[1];
      if (second == 1) {
        if (is_st_ready_pub_) {
          std_msgs::msg::UInt8 msg;
          msg.data = 1;
          is_st_ready_pub_->publish(msg);
          RCLCPP_INFO(node_->get_logger(), "Published /is_st_ready: 1 (from 0xF1 frame)");
        }
        // sendA2ReadyResponse();
        has_st_ready_pub_ = true;
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "0xF1 frame second byte != 1 (%u), ignoring", static_cast<unsigned>(second));
      }
      break;
    }
    case 0x05: {
      if (data.size() < 2) {
        RCLCPP_WARN(node_->get_logger(), "protocolDataHandler: ID 0x05 data too short");
        break;
      }
      const int16_t raw_value = static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8));
      // 高度上限钳位已取消：原始测高值直接发布，异常值的滤波交给下游
      // （面阵激光 laser_array_ground_node 的空间/时间滤波）处理。
      const int16_t value = raw_value;
      std_msgs::msg::Int16 msg;
      msg.data = value;
      // STM32 单点激光原值始终发到 /height_raw_stm32(诊断/对比用,不受标志位影响)
      if (height_raw_stm32_pub_) {
        height_raw_stm32_pub_->publish(msg);
      }
      // 高度源=STM32 时才占据 /height;=面阵时 /height 由面阵回调发布,这里不发以免两源打架
      if (!use_laser_array_height_ && height_pub_) {
        height_pub_->publish(msg);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Published /height(STM32单点): %d", value);
      }
      break;
    }
    case 0xB1: {  // 飞控发送的目标速度数据
      if (data.size() < 8) {
        RCLCPP_WARN(node_->get_logger(),
                    "protocolDataHandler: ID 0xB1 data too short (expected 8, got %zu)", data.size());
        break;
      }

      int16_t vel_x = static_cast<int16_t>(data[0] | (data[1] << 8));
      int16_t vel_y = static_cast<int16_t>(data[2] | (data[3] << 8));
      int16_t vel_z = static_cast<int16_t>(data[4] | (data[5] << 8));
      int16_t yaw   = static_cast<int16_t>(data[6] | (data[7] << 8));

      // 每秒打印两次日志（500ms一次）
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "[0xB1] Target Speed -> X:%d, Y:%d, Z:%d, Yaw:%d",
        vel_x, vel_y, vel_z, yaw);

      break;
    }
    default: {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
        "Unhandled protocol ID: 0x%02X, len=%zu", id, data.size());
      break;
    }
  }
}

void UartToStm32::sendA2ReadyResponse()
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Serial port is not open, cannot send A2 ready response");
    return;
  }

  std::vector<uint8_t> data(9, 0x00);
  data[0] = 0x01;
  if (serial_comm_->send_protocol_data(A2_READY_RESP_ID, static_cast<uint8_t>(data.size()), data)) {
    RCLCPP_INFO(node_->get_logger(), "Sent A2 ready response (len=9, first=0x01)");
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to send A2 ready response: %s", serial_comm_->get_last_error().c_str());
  }
}

}  // namespace uart_to_stm32
