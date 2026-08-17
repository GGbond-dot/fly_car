// 面阵激光 —— 抗高台突变的地面高度估计节点
//
// 与 laser_array_driver.cpp 完全解耦：自带串口解析，只是在
// 16 束有效距离上跑空间 + 时间滤波，输出真正的"离地高度"，
// 同时提供障碍检测供搬运/避障逻辑使用。
//
// 发布的话题：
//   /laser_array/ground_height    (Int16,   单位: cm)  给飞控/航点的稳定高度（与飞控 /height 语义对齐）
//   /laser_array/min_range        (Float32, 单位: m)   16 束中最小值（可能打到障碍/高台顶）
//   /laser_array/obstacle_below   (Bool)               下方是否有明显低于地面的物体
//   /laser_array/raw_percentile   (Float32)            滤波前的原始分位值（调参观察用）

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <boost/asio.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace laser_array_ground {

namespace protocol {
constexpr std::uint8_t HEADER[4] = {0x57, 0x01, 0xFF, 0x00};
constexpr std::size_t HEADER_SIZE = 4;
constexpr std::size_t TIMESTAMP_SIZE = 4;
constexpr std::size_t COUNT_SIZE = 1;
constexpr std::size_t DATA_UNIT_SIZE = 6;
constexpr std::uint8_t DATA_COUNT = 16;
constexpr std::size_t PACKET_SIZE = 112;
constexpr std::size_t MAX_BUFFER_SIZE = 4096;
}  // namespace protocol

class GroundHeightNode : public rclcpp::Node {
 public:
  GroundHeightNode() : Node("laser_array_ground_node") {
    // --- 串口 ---
    declare_parameter<std::string>("serial_port", "/dev/ttyS3");
    declare_parameter<int>("baud_rate", 921600);

    // --- 空间滤波 ---
    // 无双峰时使用：1.0=最大值，0.8=80分位
    declare_parameter<double>("percentile", 1.0);
    // 相邻两束距离差超过此值认为出现"双峰"(柱子 vs 地面)，单位 m
    declare_parameter<double>("cluster_gap", 0.20);

    // --- 时间滤波 ---
    declare_parameter<double>("max_slew_rate", 1.5);
    declare_parameter<double>("ema_alpha", 0.6);
    declare_parameter<double>("jump_threshold", 0.15);
    // 当所有光束都"短"且无远簇时：最多保持上一帧输出多少帧
    // 50Hz 数据下 20 帧 ≈ 0.4s，足够飞越一个 15cm 高台
    declare_parameter<int>("max_hold_frames", 20);

    // --- 障碍检测 ---
    declare_parameter<double>("obstacle_margin", 0.20);

    // --- 日志 ---
    declare_parameter<double>("log_period_sec", 0.5);

    get_parameter("serial_port", serial_port_name_);
    get_parameter("baud_rate", baud_rate_);
    get_parameter("percentile", percentile_);
    get_parameter("cluster_gap", cluster_gap_);
    get_parameter("max_slew_rate", max_slew_rate_);
    get_parameter("ema_alpha", ema_alpha_);
    get_parameter("jump_threshold", jump_threshold_);
    get_parameter("max_hold_frames", max_hold_frames_);
    get_parameter("obstacle_margin", obstacle_margin_);
    double log_period_sec = 0.5;
    get_parameter("log_period_sec", log_period_sec);

    percentile_ = std::clamp(percentile_, 0.0, 1.0);
    ema_alpha_ = std::clamp(ema_alpha_, 0.0, 1.0);

    ground_pub_ = create_publisher<std_msgs::msg::Int16>("/laser_array/ground_height", 10);
    min_pub_ = create_publisher<std_msgs::msg::Float32>("/laser_array/min_range", 10);
    raw_pub_ = create_publisher<std_msgs::msg::Float32>("/laser_array/raw_percentile", 10);
    obstacle_pub_ = create_publisher<std_msgs::msg::Bool>("/laser_array/obstacle_below", 10);

    read_buffer_.resize(1024);
    parse_buffer_.resize(protocol::MAX_BUFFER_SIZE);

    openSerial();

    // 定时打印当前高度（默认 0.5s 一次）
    const auto period = std::chrono::milliseconds(
        static_cast<int>(std::max(0.05, log_period_sec) * 1000.0));
    log_timer_ = create_wall_timer(period, [this]() {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (!have_last_) {
        RCLCPP_INFO(get_logger(), "[height] waiting for first valid frame...");
        return;
      }
      RCLCPP_INFO(get_logger(),
                  "[height] out=%dcm raw=%dcm min=%dcm valid=%d bimodal=%d hold=%d obstacle=%d",
                  static_cast<int>(std::lround(last_output_ * 100.0f)),
                  static_cast<int>(std::lround(last_raw_    * 100.0f)),
                  static_cast<int>(std::lround(last_min_    * 100.0f)),
                  last_valid_count_,
                  last_bimodal_ ? 1 : 0, last_hold_counter_,
                  last_obstacle_ ? 1 : 0);
    });

    RCLCPP_INFO(get_logger(),
                "ground_node up: port=%s baud=%d pct=%.2f gap=%.2f slew=%.2fm/s alpha=%.2f jump=%.2fm hold=%d",
                serial_port_name_.c_str(), baud_rate_, percentile_, cluster_gap_,
                max_slew_rate_, ema_alpha_, jump_threshold_, max_hold_frames_);
  }

  ~GroundHeightNode() override {
    running_ = false;
    if (serial_ && serial_->is_open()) {
      try { serial_->close(); } catch (...) {}
    }
    if (io_thread_.joinable()) io_thread_.join();
  }

 private:
  // 参数
  std::string serial_port_name_;
  int baud_rate_{921600};
  double percentile_{1.0};
  double cluster_gap_{0.20};
  double max_slew_rate_{1.5};
  double ema_alpha_{0.6};
  double jump_threshold_{0.15};
  int max_hold_frames_{20};
  double obstacle_margin_{0.20};

  // 串口
  boost::asio::io_service io_;
  std::shared_ptr<boost::asio::serial_port> serial_;
  std::thread io_thread_;
  std::atomic<bool> running_{false};

  // 解析缓冲
  std::vector<std::uint8_t> read_buffer_;
  std::vector<std::uint8_t> parse_buffer_;
  std::size_t buf_idx_{0};
  std::mutex buf_mtx_;

  // 时间滤波状态
  std::mutex state_mtx_;
  bool have_last_{false};
  float last_output_{0.0f};
  float last_raw_{0.0f};
  float last_min_{0.0f};
  int last_valid_count_{0};
  bool last_bimodal_{false};
  bool last_obstacle_{false};
  int last_hold_counter_{0};
  int hold_counter_{0};
  rclcpp::Time last_stamp_;

  // 发布器 + 日志定时器
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ground_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
  rclcpp::TimerBase::SharedPtr log_timer_;

  void openSerial() {
    serial_ = std::make_shared<boost::asio::serial_port>(io_);
    serial_->open(serial_port_name_);
    using spb = boost::asio::serial_port_base;
    serial_->set_option(spb::baud_rate(baud_rate_));
    serial_->set_option(spb::character_size(8));
    serial_->set_option(spb::stop_bits(spb::stop_bits::one));
    serial_->set_option(spb::parity(spb::parity::none));
    serial_->set_option(spb::flow_control(spb::flow_control::none));

    running_ = true;
    io_thread_ = std::thread(&GroundHeightNode::readLoop, this);
  }

  void readLoop() {
    while (running_) {
      try {
        if (!serial_->is_open()) break;
        std::size_t n = serial_->read_some(boost::asio::buffer(read_buffer_));
        if (n > 0) {
          std::lock_guard<std::mutex> lk(buf_mtx_);
          if (buf_idx_ + n > parse_buffer_.size()) buf_idx_ = 0;
          std::memcpy(parse_buffer_.data() + buf_idx_, read_buffer_.data(), n);
          buf_idx_ += n;
          drainPackets();
        }
      } catch (const std::exception& e) {
        if (running_) {
          RCLCPP_WARN(get_logger(), "serial read err: %s", e.what());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  void drainPackets() {
    while (buf_idx_ >= protocol::PACKET_SIZE) {
      bool found = false;
      for (std::size_t i = 0; i + protocol::PACKET_SIZE <= buf_idx_; ++i) {
        if (std::memcmp(parse_buffer_.data() + i, protocol::HEADER,
                        protocol::HEADER_SIZE) != 0) continue;
        std::uint8_t cnt = parse_buffer_[i + protocol::HEADER_SIZE + protocol::TIMESTAMP_SIZE];
        if (cnt != protocol::DATA_COUNT) continue;
        std::uint8_t cksum = 0;
        for (std::size_t k = 0; k < protocol::PACKET_SIZE - 1; ++k)
          cksum += parse_buffer_[i + k];
        if (cksum != parse_buffer_[i + protocol::PACKET_SIZE - 1]) continue;

        handlePacket(parse_buffer_.data() + i);
        shift(i + protocol::PACKET_SIZE);
        found = true;
        break;
      }
      if (!found) { shift(1); break; }
    }
  }

  void shift(std::size_t off) {
    if (off >= buf_idx_) { buf_idx_ = 0; return; }
    std::memmove(parse_buffer_.data(), parse_buffer_.data() + off, buf_idx_ - off);
    buf_idx_ -= off;
  }

  static float decodeDistance(const std::uint8_t* p) {
    std::int32_t raw = p[0] | (p[1] << 8) | (p[2] << 16);
    if (raw & 0x800000) raw |= 0xFF000000;
    return static_cast<float>(raw) / 1000000.0f;
  }

  void handlePacket(const std::uint8_t* pkt) {
    std::vector<float> valid;
    valid.reserve(protocol::DATA_COUNT);
    for (int i = 0; i < protocol::DATA_COUNT; ++i) {
      std::size_t off = protocol::HEADER_SIZE + protocol::TIMESTAMP_SIZE +
                        protocol::COUNT_SIZE + i * protocol::DATA_UNIT_SIZE;
      if (pkt[off + 3] == 0) valid.push_back(decodeDistance(pkt + off));
    }

    if (valid.empty()) {
      // 无有效束：跳过这一帧，不发布 ground_height（Int16 无 NaN 表达）
      return;
    }

    processAndPublish(valid);
  }

  void processAndPublish(std::vector<float>& valid) {
    std::sort(valid.begin(), valid.end());
    const std::size_t n = valid.size();
    const float min_range = valid.front();

    // --- 空间滤波：找最大间隙判断是否双峰 ---
    // 如果有柱子在下方但还有部分光束打到地面，距离值会分成"近簇(柱面)"
    // 和"远簇(地面)"两堆，中间有一个明显间隙。
    float max_gap = 0.0f;
    std::size_t split_idx = n;  // [split_idx, n) 为远簇
    for (std::size_t i = 1; i < n; ++i) {
      float gap = valid[i] - valid[i - 1];
      if (gap > max_gap) {
        max_gap = gap;
        split_idx = i;
      }
    }
    const bool bimodal = (max_gap > static_cast<float>(cluster_gap_)) &&
                         split_idx < n;

    // 空间滤波：永远取最大值作为地面估计。
    // 理由：ToF 传感器几乎不会"测长了"，正常失败是 invalid 或短读数；
    //       柱子/障碍只会让部分光束变短；所以 max(valid) 就是真·地面。
    // 双峰检测保留下来仅用于 obstacle 标志 和 "全遮挡冻结" 逻辑。
    const float raw_ground = valid.back();
    (void)percentile_;  // 保留参数兼容，不再参与计算

    publishFloat(raw_pub_, raw_ground);
    publishFloat(min_pub_, min_range);

    // 障碍检测：有双峰 或 最小束显著近于"地面"
    const bool obstacle = bimodal ||
                          (raw_ground - min_range) > static_cast<float>(obstacle_margin_);
    std_msgs::msg::Bool ob; ob.data = obstacle; obstacle_pub_->publish(ob);

    // --- 时间滤波 ---
    const auto now = this->now();
    float output;
    int hold_ctr;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (!have_last_) {
        output = raw_ground;
        hold_counter_ = 0;
        have_last_ = true;
      } else {
        double dt = (now - last_stamp_).seconds();
        if (dt <= 0.0 || dt > 1.0) dt = 0.02;

        float delta = raw_ground - last_output_;

        // 关键场景：下方被柱子完全遮挡（所有束都短、无远簇）、
        // 且 raw 比上一帧输出低很多 —— 显然不是真的掉下去，保持上一帧
        const bool suspicious_drop =
            !bimodal && delta < -static_cast<float>(jump_threshold_);

        if (suspicious_drop && hold_counter_ < max_hold_frames_) {
          output = last_output_;            // 冻结
          hold_counter_++;
        } else {
          // 正常分支：斜率限制 + 一阶低通
          const float max_delta = static_cast<float>(max_slew_rate_ * dt);
          if (delta > max_delta) delta = max_delta;
          else if (delta < -max_delta) delta = -max_delta;
          const float slew = last_output_ + delta;
          output = static_cast<float>(ema_alpha_) * slew +
                   static_cast<float>(1.0 - ema_alpha_) * last_output_;
          hold_counter_ = 0;
        }
      }

      last_output_ = output;
      last_raw_ = raw_ground;
      last_min_ = min_range;
      last_valid_count_ = static_cast<int>(n);
      last_bimodal_ = bimodal;
      last_obstacle_ = obstacle;
      last_hold_counter_ = hold_counter_;
      last_stamp_ = now;
      hold_ctr = hold_counter_;
    }
    (void)hold_ctr;
    // m → cm (Int16)，四舍五入
    std_msgs::msg::Int16 gh;
    gh.data = static_cast<int16_t>(std::lround(output * 100.0f));
    ground_pub_->publish(gh);
  }

  void publishFloat(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& pub,
                    float v) {
    std_msgs::msg::Float32 m;
    m.data = v;
    pub->publish(m);
  }
};

}  // namespace laser_array_ground

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<laser_array_ground::GroundHeightNode>());
  } catch (const std::exception& e) {
    fprintf(stderr, "ground_node fatal: %s\n", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
