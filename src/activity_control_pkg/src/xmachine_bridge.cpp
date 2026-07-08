// xmachine_bridge(飞车侧)—— 跨机信号的原生 UDP 桥。
//
// 背景:车与飞车都没设 ROS_DOMAIN_ID(都在默认域 0),且路由器基本挡了 DDS 多播
//   (位姿桥 pose_sender 走原生 UDP 正是这个原因)。若靠 DDS 单播 peer 跨机,会把
//   /scan、/tf、map 一起暴露 → 两边 Cartographer 同时跑必串台污染 TF。所以跨机信号
//   一律走原生 UDP,完全不碰 DDS 发现,scan/tf/map 天然隔离。本节点把三条跨机信号
//   在本地 ROS 话题 ↔ UDP 之间翻译(mission_sequencer / obstacle_detector 零改动):
//
//   本地订阅 → 发给车:
//     /resupply_request (Bool)          → UDP FC02  飞车到起飞点叫车
//     /detected_obstacle (Float32MultiArray) → UDP FC03  折线障碍(拟合出来发给车,不常发)
//   收车 UDP → 本地发布:
//     UDP FC04 → /resupply_done (Bool, latched)   车补给完成、已退开
//     UDP FC06 → /terminal_confirm (Bool, latched) terminal 确认放行右转投货
//     UDP FC07 → /mission_start (Bool, latched)    terminal 语音"开始救援"启动信号
//
// 一次性事件用 burst 重发扛 UDP 丢包(语义仍是"一次",接收端幂等/按 id 去重)。
// 车侧对端见 car/follower_pkg/src/xmachine_bridge.cpp —— 包格式改一处必须两边同步改。

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace
{
constexpr uint16_t kMagicReq  = 0xFC02;  // 飞车→车:resupply_request
constexpr uint16_t kMagicObst = 0xFC03;  // 飞车→车:折线障碍
constexpr uint16_t kMagicDone = 0xFC04;  // 车→飞车:resupply_done
constexpr uint16_t kMagicRescuee = 0xFC05;  // 飞车→车:YOLO 识别到 rescuee(value=类别 1/2)
constexpr uint16_t kMagicConfirm = 0xFC06;  // 车→飞车:terminal 确认(放行右转投货)
constexpr uint16_t kMagicStart   = 0xFC07;  // 车→飞车:terminal"开始救援"启动信号
constexpr std::size_t kMaxObstFloats = 256;  // 折线数组上限(2N+3,N 顶点),约束缓冲区

struct __attribute__((packed)) BoolPacket   // REQ / DONE
{
  uint16_t magic;
  uint16_t id;      // 事件 id(接收端去重);同一次事件的 burst 内 id 相同
  uint8_t  value;   // 1
};
static_assert(sizeof(BoolPacket) == 5, "BoolPacket must be 5 bytes");

struct __attribute__((packed)) ObstHeader   // OBST,后跟 count 个 float32
{
  uint16_t magic;
  uint16_t id;      // 拟合序号,每次新折线 +1;接收端据此当"一次"
  uint16_t count;   // float 个数
};
static_assert(sizeof(ObstHeader) == 6, "ObstHeader must be 6 bytes");
}  // namespace

class XMachineBridge : public rclcpp::Node
{
public:
  XMachineBridge()
  : Node("xmachine_bridge")
  {
    car_ip_        = declare_parameter<std::string>("car_ip", "192.168.10.161");
    to_car_port_   = declare_parameter<int>("to_car_port", 8890);    // 车侧 bind 这个口收
    from_car_port_ = declare_parameter<int>("from_car_port", 8891);  // 本节点 bind 这个口收车的
    send_hz_       = declare_parameter<double>("send_hz", 10.0);
    resend_count_  = declare_parameter<int>("resend_count", 30);     // 每次事件重发包数(~3s@10Hz)
    yolo_conf_thresh_ = declare_parameter<double>("yolo_conf_thresh", 0.25);  // 判"识别到"的最低置信度
    yolo_debounce_    = declare_parameter<int>("yolo_debounce_frames", 2);    // 连续几帧命中才算(去抖)
    const std::string det_topic = declare_parameter<std::string>(
      "yolo_detections_topic", "/yolo_detector/detections");

    // --- 发送 socket(到车)---
    tx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_fd_ < 0) { throw std::runtime_error("xmachine_bridge: tx socket 创建失败"); }
    std::memset(&car_addr_, 0, sizeof(car_addr_));
    car_addr_.sin_family = AF_INET;
    car_addr_.sin_port = htons(static_cast<uint16_t>(to_car_port_));
    if (::inet_pton(AF_INET, car_ip_.c_str(), &car_addr_.sin_addr) != 1) {
      ::close(tx_fd_);
      throw std::runtime_error("xmachine_bridge: car_ip 非法: " + car_ip_);
    }

    // --- 接收 socket(收车的 done)---
    rx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_fd_ < 0) { ::close(tx_fd_); throw std::runtime_error("xmachine_bridge: rx socket 创建失败"); }
    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(static_cast<uint16_t>(from_car_port_));
    if (::bind(rx_fd_, reinterpret_cast<sockaddr *>(&bind_addr), sizeof(bind_addr)) < 0) {
      ::close(tx_fd_); ::close(rx_fd_);
      throw std::runtime_error("xmachine_bridge: bind 端口失败 " + std::to_string(from_car_port_));
    }
    const int flags = ::fcntl(rx_fd_, F_GETFL, 0);
    ::fcntl(rx_fd_, F_SETFL, flags | O_NONBLOCK);

    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    req_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/resupply_request", latched,
      std::bind(&XMachineBridge::onRequest, this, std::placeholders::_1));
    obst_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/detected_obstacle", latched,
      std::bind(&XMachineBridge::onObstacle, this, std::placeholders::_1));
    // YOLO 检测流:volatile 默认 QoS(与 yolo_detector 发布端一致,不用 latched)
    det_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      det_topic, rclcpp::QoS(10),
      std::bind(&XMachineBridge::onDetections, this, std::placeholders::_1));
    done_pub_ = create_publisher<std_msgs::msg::Bool>("/resupply_done", latched);
    confirm_pub_ = create_publisher<std_msgs::msg::Bool>("/terminal_confirm", latched);
    start_pub_ = create_publisher<std_msgs::msg::Bool>("/mission_start", latched);  // terminal"开始救援"→ mission

    const auto period = std::chrono::duration<double>(1.0 / std::max(send_hz_, 1.0));
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&XMachineBridge::sendTick, this));
    rx_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&XMachineBridge::pollRx, this));

    RCLCPP_INFO(get_logger(),
      "xmachine_bridge(飞车):发车 %s:%d,收车 :%d。req/obst/rescuee→车,done/confirm←车。",
      car_ip_.c_str(), to_car_port_, from_car_port_);
  }

  ~XMachineBridge() override
  {
    if (tx_fd_ >= 0) { ::close(tx_fd_); }
    if (rx_fd_ >= 0) { ::close(rx_fd_); }
  }

private:
  void onRequest(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      req_id_++;
      req_remaining_ = resend_count_;
      RCLCPP_INFO(get_logger(), "收到 /resupply_request,开始向车重发 REQ(id=%u)", req_id_);
    }
  }

  void onObstacle(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    obst_payload_.assign(msg->data.begin(), msg->data.end());
    if (obst_payload_.size() > kMaxObstFloats) { obst_payload_.resize(kMaxObstFloats); }
    obst_id_++;
    obst_remaining_ = resend_count_;
    RCLCPP_INFO(get_logger(), "收到 /detected_obstacle(%zu float),开始向车重发 OBST(id=%u)",
      obst_payload_.size(), obst_id_);
  }

  // YOLO 检测:每 6 个 float 一个目标 [cls,conf,x1,y1,x2,y2]。取置信度最高、≥阈值的 rescuee,
  // 连续 yolo_debounce_ 帧命中同类才判"识别到",持续向车重发 RESCUEE(value=类别 1/2)。
  void onDetections(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    int best_cls = -1;
    double best_conf = yolo_conf_thresh_;
    for (std::size_t i = 0; i + 5 < msg->data.size(); i += 6) {
      const int cls = static_cast<int>(msg->data[i]);
      const double conf = msg->data[i + 1];
      if (conf >= best_conf) { best_conf = conf; best_cls = cls; }
    }
    const uint8_t cls_val = (best_cls >= 0) ? static_cast<uint8_t>(best_cls + 1) : 0;  // 1=rescuee1,2=rescuee2

    if (cls_val != 0 && cls_val == pending_class_) {
      hit_count_++;
    } else {
      pending_class_ = cls_val;
      hit_count_ = (cls_val != 0) ? 1 : 0;
    }
    // 命中去抖帧数 → 持续重发(每帧命中都续上 burst,YOLO 一直看到就一直发)
    if (cls_val != 0 && hit_count_ >= yolo_debounce_) {
      if (rescuee_class_ != cls_val) {
        rescuee_id_++;
        RCLCPP_INFO(get_logger(), "YOLO 识别到 rescuee%u,开始向车重发 RESCUEE", cls_val);
      }
      rescuee_class_ = cls_val;
      rescuee_remaining_ = resend_count_;
    }
  }

  void sendTick()
  {
    if (req_remaining_ > 0) {
      BoolPacket pkt{kMagicReq, req_id_, 1};
      ::sendto(tx_fd_, &pkt, sizeof(pkt), 0,
        reinterpret_cast<const sockaddr *>(&car_addr_), sizeof(car_addr_));
      req_remaining_--;
    }
    if (obst_remaining_ > 0) {
      std::vector<uint8_t> buf(sizeof(ObstHeader) + obst_payload_.size() * sizeof(float));
      ObstHeader hdr{kMagicObst, obst_id_, static_cast<uint16_t>(obst_payload_.size())};
      std::memcpy(buf.data(), &hdr, sizeof(hdr));
      if (!obst_payload_.empty()) {
        std::memcpy(buf.data() + sizeof(hdr), obst_payload_.data(),
          obst_payload_.size() * sizeof(float));
      }
      ::sendto(tx_fd_, buf.data(), buf.size(), 0,
        reinterpret_cast<const sockaddr *>(&car_addr_), sizeof(car_addr_));
      obst_remaining_--;
    }
    if (rescuee_remaining_ > 0) {
      BoolPacket pkt{kMagicRescuee, rescuee_id_, rescuee_class_};   // value=类别 1/2
      ::sendto(tx_fd_, &pkt, sizeof(pkt), 0,
        reinterpret_cast<const sockaddr *>(&car_addr_), sizeof(car_addr_));
      rescuee_remaining_--;
    }
  }

  void pollRx()
  {
    BoolPacket pkt;
    while (true) {
      const ssize_t n = ::recv(rx_fd_, &pkt, sizeof(pkt), 0);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) { break; }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "recv 失败: %s", std::strerror(errno));
        break;
      }
      if (n != static_cast<ssize_t>(sizeof(pkt))) { continue; }
      if (pkt.magic == kMagicDone && !done_published_) {
        done_published_ = true;
        std_msgs::msg::Bool m; m.data = true;
        done_pub_->publish(m);
        RCLCPP_INFO(get_logger(), "收到车 UDP done,本地发 /resupply_done=1");
      } else if (pkt.magic == kMagicConfirm && !confirm_published_) {
        confirm_published_ = true;
        std_msgs::msg::Bool m; m.data = true;
        confirm_pub_->publish(m);
        RCLCPP_INFO(get_logger(), "收到车 UDP confirm,本地发 /terminal_confirm=1");
      } else if (pkt.magic == kMagicStart && !start_published_) {
        start_published_ = true;
        std_msgs::msg::Bool m; m.data = true;
        start_pub_->publish(m);
        RCLCPP_INFO(get_logger(), "收到车 UDP start,本地发 /mission_start=1('开始救援')");
      }
    }
  }

  std::string car_ip_;
  int to_car_port_, from_car_port_, resend_count_;
  double send_hz_;
  double yolo_conf_thresh_;
  int yolo_debounce_;

  int tx_fd_{-1}, rx_fd_{-1};
  sockaddr_in car_addr_{};

  uint16_t req_id_{0}, obst_id_{0}, rescuee_id_{0};
  int req_remaining_{0}, obst_remaining_{0}, rescuee_remaining_{0};
  std::vector<float> obst_payload_;
  bool done_published_{false};
  bool confirm_published_{false};
  bool start_published_{false};
  // YOLO 去抖 + 当前判定
  uint8_t pending_class_{0}, rescuee_class_{0};
  int hit_count_{0};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr req_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obst_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr det_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr confirm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
  rclcpp::TimerBase::SharedPtr send_timer_, rx_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XMachineBridge>());
  rclcpp::shutdown();
  return 0;
}
