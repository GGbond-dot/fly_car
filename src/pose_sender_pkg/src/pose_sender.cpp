// pose_sender — 飞车位姿 UDP 发送(供地面车跟随,见 car/docs/follow_fly_car_design.md §2.1/§4.1)
//
// 定频查 TF map←laser_link,打包成 24 字节 UDP 包单播给车。
// 包格式(小端,与 car 侧 follower_pkg/leader_pose_receiver.cpp 的 PosePacket 保持一致,改一处必须同步改另一处):
//   [magic u16 = 0xFC01][seq u16][stamp_ms u32][x_m f32][y_m f32][yaw_rad f32][reserved f32]
//
// 不依赖 DDS 跨机发现;丢包不重传,下一包就是最新状态。

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr uint16_t kMagic = 0xFC01;

struct __attribute__((packed)) PosePacket
{
  uint16_t magic;
  uint16_t seq;
  uint32_t stamp_ms;
  float x_m;
  float y_m;
  float yaw_rad;
  float reserved;
};
static_assert(sizeof(PosePacket) == 24, "PosePacket must be 24 bytes");
}  // namespace

class PoseSender : public rclcpp::Node
{
public:
  PoseSender()
  : Node("pose_sender")
  {
    declare_parameter<std::string>("target_ip", "192.168.4.2");  // 车的 IP,按组网实配!
    declare_parameter<int>("target_port", 8888);
    declare_parameter<double>("rate_hz", 20.0);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("body_frame", "laser_link");

    const std::string target_ip = get_parameter("target_ip").as_string();
    const int target_port = get_parameter("target_port").as_int();
    const double rate_hz = get_parameter("rate_hz").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    body_frame_ = get_parameter("body_frame").as_string();

    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
      throw std::runtime_error("pose_sender: failed to create UDP socket");
    }
    std::memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(static_cast<uint16_t>(target_port));
    if (::inet_pton(AF_INET, target_ip.c_str(), &dest_addr_.sin_addr) != 1) {
      ::close(sock_fd_);
      throw std::runtime_error("pose_sender: invalid target_ip: " + target_ip);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const double period_sec = 1.0 / std::max(rate_hz, 1.0);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_sec)),
      std::bind(&PoseSender::timerCallback, this));

    RCLCPP_INFO(get_logger(), "pose_sender: %s:%d @ %.0fHz, TF %s<-%s",
      target_ip.c_str(), target_port, rate_hz, map_frame_.c_str(), body_frame_.c_str());
  }

  ~PoseSender() override
  {
    if (sock_fd_ >= 0) {
      ::close(sock_fd_);
    }
  }

private:
  void timerCallback()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, body_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "pose_sender: TF %s<-%s unavailable: %s", map_frame_.c_str(), body_frame_.c_str(), ex.what());
      return;
    }

    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    PosePacket pkt;
    pkt.magic = kMagic;
    pkt.seq = seq_++;
    pkt.stamp_ms = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    pkt.x_m = static_cast<float>(tf.transform.translation.x);
    pkt.y_m = static_cast<float>(tf.transform.translation.y);
    pkt.yaw_rad = static_cast<float>(yaw);
    pkt.reserved = 0.0f;

    const ssize_t sent = ::sendto(sock_fd_, &pkt, sizeof(pkt), 0,
      reinterpret_cast<const sockaddr *>(&dest_addr_), sizeof(dest_addr_));
    if (sent != static_cast<ssize_t>(sizeof(pkt))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "pose_sender: sendto failed: %s", std::strerror(errno));
    }
  }

  std::string map_frame_;
  std::string body_frame_;
  int sock_fd_{-1};
  sockaddr_in dest_addr_{};
  uint16_t seq_{0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSender>());
  rclcpp::shutdown();
  return 0;
}
