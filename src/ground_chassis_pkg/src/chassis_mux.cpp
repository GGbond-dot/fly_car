// chassis_mux — 地面/空中仲裁节点(两套控制链互斥的唯一裁判)
//
// 模式判据(非对称,保证起飞快、降落稳):
//   起飞 GROUND→AIR:目标航点 z > z_threshold_cm 立即切空中(让飞控马上接管拔高)。
//   降落 AIR→GROUND:必须目标 z ≤ 阈值【且】实测 /height ≤ land_height_cm 才切地面——
//     否则降落航点(目标 z=4 但飞机还在 100cm)会按目标 z 误切地面,
//     导致飞控被关、轮子在空中空转、飞机失控掉落。所以降落看“真实高度”。
//   规则统一为:AIR 当 (目标z>阈值) 或 (实测高度>land_height_cm);否则 GROUND。
//
// latched 发布两个互斥使能,由同一 mode 派生,绝不会两套同时为真:
//   /ground_enable → diff_drive_controller;  /flight_enable → pid_control_pkg(标志位)
// 启动至收到首个目标前为安全态(两者皆 false)。

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"

class ChassisMux : public rclcpp::Node
{
public:
  ChassisMux()
  : Node("chassis_mux")
  {
    z_threshold_cm_ = declare_parameter<double>("z_threshold_cm", 20.0);
    land_height_cm_ = declare_parameter<double>("land_height_cm", 15.0);  // 实测高度低于此才允许切回地面
    settle_s_ = declare_parameter<double>("settle_s", 2.0);  // 起飞/降落切换前的稳定停顿时长

    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    ground_enable_pub_ = create_publisher<std_msgs::msg::Bool>("/ground_enable", latched);
    flight_enable_pub_ = create_publisher<std_msgs::msg::Bool>("/flight_enable", latched);

    // 订阅 /target_position(匹配 RouteTargetPublisher 的 transient_local 发布)
    target_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_position", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&ChassisMux::targetCallback, this, std::placeholders::_1));

    // 订阅实测高度(uart_to_stm32 发,飞控回传),用于降落判据
    height_sub_ = create_subscription<std_msgs::msg::Int16>(
      "/height", rclcpp::QoS(10),
      std::bind(&ChassisMux::heightCallback, this, std::placeholders::_1));

    // 启动安全态:两者皆 false
    publishEnables(false, false);

    RCLCPP_INFO(get_logger(),
      "chassis_mux up: z_threshold=%.1fcm land_height=%.1fcm settle=%.1fs; 起飞看目标z,降落看实测高度,切换前停 settle 秒",
      z_threshold_cm_, land_height_cm_, settle_s_);
  }

private:
  enum class Mode { SAFE, GROUND, AIR };

  void targetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      return;
    }
    last_target_z_cm_ = static_cast<double>(msg->data[2]);
    has_target_ = true;
    recomputeMode();
  }

  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    current_height_cm_ = static_cast<double>(msg->data);
    has_height_ = true;
    recomputeMode();
  }

  void recomputeMode()
  {
    if (!has_target_) {
      return;  // 还没目标,维持启动安全态
    }
    // AIR 当:目标命令上天,或飞机实测还在空中(降落途中)
    const bool commanded_air = last_target_z_cm_ > z_threshold_cm_;
    const bool physically_air = has_height_ && current_height_cm_ > land_height_cm_;
    const Mode desired = (commanded_air || physically_air) ? Mode::AIR : Mode::GROUND;

    if (settling_) {
      if (desired != pending_mode_) {
        // 缓冲期内目标又变了:改向新目标,重新计时(期间仍两者皆停)
        pending_mode_ = desired;
        startSettleTimer();
        RCLCPP_INFO(get_logger(), "切换缓冲期内目标变更 → 重新缓冲 %.1fs", settle_s_);
      }
      return;
    }
    if (desired == active_mode_) {
      return;  // 模式没变
    }

    // 启动首次激活(SAFE→x):无需缓冲,直接生效
    if (active_mode_ == Mode::SAFE) {
      applyMode(desired);
      return;
    }

    // 起飞/降落切换:先停 settle_s 秒(两者皆 false),再切到目标模式
    settling_ = true;
    pending_mode_ = desired;
    publishEnables(false, false);
    startSettleTimer();
    RCLCPP_INFO(get_logger(),
      "进入切换缓冲:目标→%s,先停 %.1fs(轮子停、飞控不发)再切",
      desired == Mode::AIR ? "飞控(起飞)" : "地面(落地)", settle_s_);
  }

  void startSettleTimer()
  {
    if (settle_timer_) {
      settle_timer_->cancel();
    }
    settle_timer_ = create_wall_timer(
      std::chrono::duration<double>(settle_s_),
      std::bind(&ChassisMux::onSettleDone, this));
  }

  void onSettleDone()
  {
    if (settle_timer_) {
      settle_timer_->cancel();
    }
    settling_ = false;
    applyMode(pending_mode_);
  }

  void applyMode(Mode mode)
  {
    active_mode_ = mode;
    const bool ground = (mode == Mode::GROUND);
    const bool flight = (mode == Mode::AIR);
    publishEnables(ground, flight);
    RCLCPP_INFO(get_logger(),
      "切换完成 → %s态:ground_enable=%d flight_enable=%d (目标z=%.1f 实测高度=%.1f)",
      ground ? "地面" : "飞控", ground, flight,
      last_target_z_cm_, has_height_ ? current_height_cm_ : -1.0);
  }

  // 互斥不变量:ground 与 flight 不可能同时为真(由 Mode 派生,这里再断言一次)
  void publishEnables(bool ground, bool flight)
  {
    if (ground && flight) {
      RCLCPP_ERROR(get_logger(), "BUG: ground 与 flight 同时为真,强制全 false");
      ground = false;
      flight = false;
    }
    std_msgs::msg::Bool g;
    g.data = ground;
    ground_enable_pub_->publish(g);
    std_msgs::msg::Bool f;
    f.data = flight;
    flight_enable_pub_->publish(f);
  }

  double z_threshold_cm_;
  double land_height_cm_;
  double settle_s_;

  Mode active_mode_{Mode::SAFE};   // 当前已生效模式
  Mode pending_mode_{Mode::SAFE};  // 缓冲期内将要切到的模式
  bool settling_{false};           // 是否处于切换缓冲(两者皆停)

  double last_target_z_cm_{0.0};
  double current_height_cm_{0.0};
  bool has_target_{false};
  bool has_height_{false};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ground_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flight_enable_pub_;
  rclcpp::TimerBase::SharedPtr settle_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisMux>());
  rclcpp::shutdown();
  return 0;
}
