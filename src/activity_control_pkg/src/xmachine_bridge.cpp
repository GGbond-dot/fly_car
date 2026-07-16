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
//     UDP FC09 → /coverage_area (Float32MultiArray, latched) 地面站选区后覆盖开跑(飞车走 L 形)
//     UDP FC0A → /wildlife/waypoints (Float32MultiArray, latched) terminal 规划的巡航航点,
//        本地(域1)重发给飞行节点 wildlife_patrol_task_node
//     UDP FC0E → /rescue/flight_search_enable (Bool) terminal 语音确认放行空中搜救起飞
//   再本地订阅 → 发给车:
//     /wildlife/status (String) → UDP FC0B  飞行节点状态回传 terminal(飞车板域1,车板域0)
//   本地查 TF → 发给车:
//     到达飞行起点 A4B8 → UDP FC0D  车侧转 /rescue/flight_launch_arrived,terminal 据此播报
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
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
constexpr uint16_t kMagicReq  = 0xFC02;  // 飞车→车:resupply_request
constexpr uint16_t kMagicObst = 0xFC03;  // 飞车→车:折线障碍
constexpr uint16_t kMagicDone = 0xFC04;  // 车→飞车:resupply_done
constexpr uint16_t kMagicRescuee = 0xFC05;  // 飞车→车:YOLO 识别到 rescuee(value=类别 1/2)
constexpr uint16_t kMagicConfirm = 0xFC06;  // 车→飞车:terminal 确认(放行右转投货)
constexpr uint16_t kMagicStart   = 0xFC07;  // 车→飞车:terminal"开始救援"启动信号
constexpr uint16_t kMagicCoverage = 0xFC09;  // 车→飞车:地面站选区后"覆盖开跑"(飞车走 L 形)
constexpr uint16_t kMagicWaypoints = 0xFC0A;  // 车→飞车:巡航航点(Float32MultiArray,4/航点)
constexpr uint16_t kMagicStatus    = 0xFC0B;  // 飞车→车:/wildlife/status(String JSON)
// 飞车→车:本机当前位置(map 系 cm)。/slam 页拿它决定点云露出到哪。
// **跨机绝不用 DDS** —— 跨设备订 odom/点云出过问题;位置在本地(同板同域)查 TF,
// 只把 12 字节结果裸 UDP 发过去,1Hz。要多省有多省。
constexpr uint16_t kMagicPose      = 0xFC0C;
constexpr uint16_t kMagicLaunchArrived = 0xFC0D;  // 飞车→车:已到飞行起点 A4B8
// 车→飞车:terminal 语音确认后放行起飞。**别改回 FC0C** —— Tier0 初稿占的就是 FC0C,
// 那是从没有 pose 的旧分支写的;FC0C 归位置回传,这个是 FC0E。
constexpr uint16_t kMagicFlightEnable = 0xFC0E;
constexpr uint16_t kMagicWaypointsAck = 0xFC0F;  // 飞车→车:已收 FC0A,id 与航点包一致
constexpr uint16_t kMagicAirborne = 0xFC10;  // 飞车→车:本地测高确认已离地(只发标志)
constexpr uint16_t kMagicAirborneAck = 0xFC11;  // 车→飞车:已收 FC10,id 与事件一致
constexpr std::size_t kMaxObstFloats = 256;  // 折线/航点数组上限(共用),约束缓冲区
constexpr std::size_t kMaxStatusBytes = 1024; // 状态 JSON 字节上限
constexpr int kStatusBurst = 4;               // 每次状态回调发几包(心跳 1Hz,小簇扛丢包)

struct __attribute__((packed)) BoolPacket   // REQ / DONE
{
  uint16_t magic;
  uint16_t id;      // 事件 id(接收端去重);同一次事件的 burst 内 id 相同
  uint8_t  value;   // 1
};
static_assert(sizeof(BoolPacket) == 5, "BoolPacket must be 5 bytes");

// POSE:定长,没有数组。与车侧 follower_pkg/src/xmachine_bridge.cpp 的 PosePacket 一致,
// 改一处两侧都要改。
struct __attribute__((packed)) PosePacket
{
  uint16_t magic;
  uint16_t id;
  float x_cm;
  float y_cm;
};
static_assert(sizeof(PosePacket) == 12, "PosePacket must be 12 bytes");

struct __attribute__((packed)) ObstHeader   // OBST / WAYPOINTS,后跟 count 个 float32
{
  uint16_t magic;
  uint16_t id;      // 拟合序号,每次新折线 +1;接收端据此当"一次"
  uint16_t count;   // float 个数
};
static_assert(sizeof(ObstHeader) == 6, "ObstHeader must be 6 bytes");

struct __attribute__((packed)) StrHeader    // STATUS,后跟 len 字节 UTF-8(JSON)
{
  uint16_t magic;
  uint16_t id;
  uint16_t len;
};
static_assert(sizeof(StrHeader) == 6, "StrHeader must be 6 bytes");
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
    // 位置回传 1Hz —— /slam 页本来就是 1Hz 轮询,发更快纯属浪费。0 = 关掉。
    pose_hz_       = declare_parameter<double>("pose_hz", 1.0);
    map_frame_     = declare_parameter<std::string>("map_frame", "map");
    laser_link_frame_ = declare_parameter<std::string>("laser_link_frame", "laser_link");

    // ---- 场地系 ↔ map 系:飞车开机怎么摆,决定了两者差多少 ----
    //
    // carto 的 map 系是**跟着开机姿态走的**:原点=开机位置,+x=开机时的车头方向。
    // 所以飞车摆的朝向一变,map 系整个跟着转,而 terminal 规划出来的航点是**场地系**的
    // (A1B1 中心=(0,0),+x 指向 x 轴最远格)—— 两者差一个旋转,不转就全歪。
    //
    // start_heading_field_deg = 飞车开机时车头在**场地系**里指向哪:
    //     0   = 朝场地 +x(老摆法,map 系 == 场地系,不做任何转换)
    //   -90   = 朝场地 -y(2026-07-16 改的新摆法)
    //
    // 为什么改摆法:地面段第一个航点是 (0,0) yaw=-90,即"原地转 90° 面朝 -y"才起步 ——
    // 而铁律是移动和转 yaw 绝不同时,那就白白多转一次。把车摆成一开始就朝 -y,
    // 转换后第一个航点的 yaw 正好变成 0(= 开机姿态),那个原地转自然消失。
    //
    // 旋转量 α:场地系转 α 得到 map 系,即 p_map = R(α)·p_field、yaw_map = yaw_field + α。
    // 车头方向在场地系是 start_heading、在 map 系恒为 0,代进去得 α = -start_heading。
    start_heading_field_deg_ = declare_parameter<double>("start_heading_field_deg", 0.0);
    field_to_map_deg_ = -start_heading_field_deg_;
    // 飞车起点在场地系的位置(米)。carto 的 map 原点=飞车开机点,所以场地→map 除了旋转
    // 还要先**减掉这个起点**:p_map = R(α)·(p_field - 起点)。飞车朝场地 x 轴(α=0)时退化成
    // 纯平移(减起点)。默认 (0.25, -0.75)m = 场地 (25,-75)cm。
    field_origin_x_m_ = declare_parameter<double>("field_origin_x_m", 0.25);
    field_origin_y_m_ = declare_parameter<double>("field_origin_y_m", -0.75);
    const double a = field_to_map_deg_ * M_PI / 180.0;
    cos_a_ = std::cos(a);
    sin_a_ = std::sin(a);
    if (start_heading_field_deg_ != 0.0) {
      RCLCPP_INFO(get_logger(),
        "飞车开机朝向 = 场地系 %.1f°,场地系→map 系转 %.1f°(航点/难民点/起飞点都会换算,"
        "位置回传换算回场地系)", start_heading_field_deg_, field_to_map_deg_);
    }
    resend_count_  = declare_parameter<int>("resend_count", 30);     // 每次事件重发包数(~3s@10Hz)
    airborne_height_cm_ = declare_parameter<int>("airborne_height_cm", 60);
    airborne_required_samples_ = declare_parameter<int>("airborne_required_samples", 2);
    // 飞行起点 A4B8(map 系,米)。**坐标待实测校准** —— 写成参数就是为了不用重编译改。
    launch_x_m_    = declare_parameter<double>("launch_target_x_m", 3.50);
    launch_y_m_    = declare_parameter<double>("launch_target_y_m", -1.50);
    launch_tol_m_  = declare_parameter<double>("launch_tol_m", 0.15);   // 到点容差
    // 只认"稳稳停住"的到达:carto 位姿会抖,擦一下容差就回传会让 terminal 早播报。
    launch_stable_s_ = declare_parameter<double>("launch_stable_s", 0.5);
    // TF 停更(carto 挂了)时旧位姿会一直躺在 buffer 里,拿它判到达等于瞎猜。
    launch_tf_fresh_s_ = declare_parameter<double>("launch_tf_fresh_s", 0.5);
    launch_check_hz_ = declare_parameter<double>("launch_check_hz", 10.0);  // 0 = 关掉到达检测
    // 难民点(map 系,米):飞到这儿就发 FC05,车侧 terminal 播报"要投放物资吗"。
    // 难民位置是**写死**的(演示用),但走的是跟 YOLO 完全一样的 FC05 链路 ——
    // terminal 那边分不出是认出来的还是算出来的。
    // **坐标待实测,所以默认 rescuee_check_hz=0(关掉)**;给了坐标再在 launch 里打开。
    rescuee_x_m_    = declare_parameter<double>("rescuee_x_m", 0.0);
    rescuee_y_m_    = declare_parameter<double>("rescuee_y_m", 0.0);
    rescuee_tol_m_  = declare_parameter<double>("rescuee_tol_m", 0.25);
    rescuee_stable_s_ = declare_parameter<double>("rescuee_stable_s", 0.3);
    rescuee_point_class_ = declare_parameter<int>("rescuee_point_class", 2);  // 1/2 = 类别
    rescuee_check_hz_ = declare_parameter<double>("rescuee_check_hz", 0.0);
    // 起飞点/难民点这两个参数填的是**场地系**坐标(跟 terminal 的 planner 同一套),
    // 而下面的到达检测是拿 map 系的 TF 比的 —— 这里一次性换算,检测里就不用再管了。
    fieldToMap(launch_x_m_, launch_y_m_, launch_x_m_, launch_y_m_);
    fieldToMap(rescuee_x_m_, rescuee_y_m_, rescuee_x_m_, rescuee_y_m_);

    yolo_conf_thresh_ = declare_parameter<double>("yolo_conf_thresh", 0.25);  // 判"识别到"的最低置信度
    yolo_debounce_    = declare_parameter<int>("yolo_debounce_frames", 2);    // 连续几帧命中才算(去抖)
    // 固定 Demo 只允许“到达写死难民点”触发 FC05。视频/画框仍照常工作。
    yolo_trigger_rescuee_event_ =
      declare_parameter<bool>("yolo_trigger_rescuee_event", false);
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
    // 覆盖开跑信号 → 本地发 /coverage_area(latched),触发 coverage_generator(L 形)。
    // l_path 模式只当开跑信号用,发 [0,0,0,0] 占位(几何用 l_* 参数)。
    coverage_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/coverage_area", latched);
    // 车侧转来的巡航航点(UDP FC0A)→ 本地(域1)latched 发给飞行节点。飞行节点用 reliable(10)
    // 订,latched(transient_local+reliable)兼容,且晚起也能拿到最后一条路线。
    wp_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/wildlife/waypoints", latched);
    // 飞行节点状态(/wildlife/status,飞行节点用 reliable+transient_local 发)→ 桥转 UDP FC0B 回车。
    status_sub_ = create_subscription<std_msgs::msg::String>(
      "/wildlife/status", latched,
      std::bind(&XMachineBridge::onStatus, this, std::placeholders::_1));
    // terminal 放行起飞(UDP FC0E)→ 本地(域1 DDS)发给飞行节点。latched:飞行节点晚起也拿得到,
    // 且这是"允许起飞"的闸门 —— 漏了就永远飞不起来,比重复发一次危险得多。
    flight_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/rescue/flight_search_enable", latched);
    // 高度只在本板判断，不跨机回传连续数据。达到阈值后仅 burst 一个 FC10 Bool 事件。
    height_sub_ = create_subscription<std_msgs::msg::Int16>(
      "/height", rclcpp::QoS(10),
      std::bind(&XMachineBridge::onHeight, this, std::placeholders::_1));

    // 位置回传和到达检测都在本地查 TF,共用一个 buffer(全关掉才不建,行为跟以前一致)。
    if (pose_hz_ > 0.0 || launch_check_hz_ > 0.0 || rescuee_check_hz_ > 0.0) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    // 位置回传:独立的低频定时器,不搭 send_hz_(10Hz)那趟车 —— 位置要的就是"最低频率"。
    if (pose_hz_ > 0.0) {
      const auto pose_period = std::chrono::duration<double>(1.0 / pose_hz_);
      pose_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(pose_period),
        std::bind(&XMachineBridge::posePick, this));
    }
    // 到达飞行起点检测:比 pose 快(要 0.5s 稳定窗),但只是本地查 TF 算距离,不发包。
    // 真到了才 burst 一次 FC0D,之后自己关掉,不会一直发。
    if (launch_check_hz_ > 0.0) {
      const auto launch_period = std::chrono::duration<double>(1.0 / launch_check_hz_);
      launch_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(launch_period),
        std::bind(&XMachineBridge::launchArrivedCheck, this));
    }
    // 飞到难民点 → FC05(跟 YOLO 同一条链路)。坐标没给就别开,免得 (0,0) 当难民点。
    if (rescuee_check_hz_ > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / rescuee_check_hz_);
      rescuee_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&XMachineBridge::rescueePointCheck, this));
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(send_hz_, 1.0));
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&XMachineBridge::sendTick, this));
    rx_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&XMachineBridge::pollRx, this));

    RCLCPP_INFO(get_logger(),
      "xmachine_bridge(飞车):发车 %s:%d,收车 :%d。"
      "req/obst/rescuee/status/pose/launch_arrived→车,"
      "done/confirm/start/coverage/waypoints/flight_enable←车。",
      car_ip_.c_str(), to_car_port_, from_car_port_);
    if (launch_check_hz_ > 0.0) {
      RCLCPP_INFO(get_logger(),
        "到达检测(FC0D): 飞行起点 A4B8=(%.2f,%.2f)m 容差 %.2fm,稳 %.1fs,TF 新鲜 %.1fs,%.0fHz 查",
        launch_x_m_, launch_y_m_, launch_tol_m_, launch_stable_s_,
        launch_tf_fresh_s_, launch_check_hz_);
    }
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

  void onHeight(const std_msgs::msg::Int16::SharedPtr msg)
  {
    if (!flight_enable_received_ || airborne_sent_) { return; }
    const int height_cm = static_cast<int>(msg->data);
    // 测高异常值绝不能放行；上游也会把 >200 的毛刺压成 2cm，这里再守一道。
    if (height_cm < airborne_height_cm_ || height_cm > 200) {
      airborne_samples_ = 0;
      return;
    }
    airborne_samples_++;
    if (airborne_samples_ < std::max(1, airborne_required_samples_)) { return; }
    airborne_sent_ = true;
    airborne_id_++;
    airborne_remaining_ = resend_count_;
    airborne_acked_ = false;
    airborne_retry_ticks_ = 0;
    RCLCPP_INFO(get_logger(),
      "测高连续 %d 次 >= %dcm(当前 %dcm) → 向车重发 AIRBORNE(FC10,id=%u)，不回传高度流",
      airborne_samples_, airborne_height_cm_, height_cm, airborne_id_);
  }

  // 飞行节点状态(1Hz 心跳 + 相位变化)。每次回调发一小簇 UDP 扛丢包;车侧按 id 去重。
  void onStatus(const std_msgs::msg::String::SharedPtr msg)
  {
    status_payload_ = msg->data;
    if (status_payload_.size() > kMaxStatusBytes) { status_payload_.resize(kMaxStatusBytes); }
    status_id_++;
    status_remaining_ = kStatusBurst;
  }

  // YOLO 检测:每 6 个 float 一个目标 [cls,conf,x1,y1,x2,y2]。取置信度最高、≥阈值的 rescuee,
  // 连续 yolo_debounce_ 帧命中同类才判"识别到",持续向车重发 RESCUEE(value=类别 1/2)。
  void onDetections(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (!yolo_trigger_rescuee_event_) {
      return;
    }
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

  // yaw 收进 (-180,180] —— 转换后可能跑到 ±270,PID 拿去算角差会绕远路。
  static double normalizeDeg(double d)
  {
    while (d > 180.0) { d -= 360.0; }
    while (d <= -180.0) { d += 360.0; }
    return d;
  }

  // ---- 场地系 ↔ map 系(见构造函数里 start_heading / field_origin 的注释)----
  // ⚠ 单位:field_origin 是**米**,所以传进来的 xf/yf 必须是米。航点是 cm,调用前 /100、调用后 *100。
  // p_map = R(α)·(p_field - 飞车起点)
  void fieldToMap(double xf, double yf, double & xm, double & ym) const
  {
    const double xf2 = xf - field_origin_x_m_;
    const double yf2 = yf - field_origin_y_m_;
    xm = xf2 * cos_a_ - yf2 * sin_a_;
    ym = xf2 * sin_a_ + yf2 * cos_a_;
  }
  // p_field = R(-α)·p_map + 飞车起点
  void mapToField(double xm, double ym, double & xf, double & yf) const
  {
    xf =  xm * cos_a_ + ym * sin_a_ + field_origin_x_m_;
    yf = -xm * sin_a_ + ym * cos_a_ + field_origin_y_m_;
  }

  // 本地查 TF 拿位置 → 12 字节 UDP 发车。位置是连续量:发一次就够,不重发
  // (丢一包等下一秒即可,重发反而是浪费)。
  void posePick()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, laser_link_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "拿不到 TF %s->%s,位置不回传: %s", map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
      return;
    }
    // 车侧 /slam 页拿它画点云,那边是场地系 —— 回传前换算回去。
    double xf, yf;
    mapToField(tf.transform.translation.x, tf.transform.translation.y, xf, yf);
    PosePacket pkt{kMagicPose, pose_id_++,
      static_cast<float>(xf * 100.0), static_cast<float>(yf * 100.0)};
    ::sendto(tx_fd_, &pkt, sizeof(pkt), 0,
      reinterpret_cast<const sockaddr *>(&car_addr_), sizeof(car_addr_));
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "位置回传中(FC0C, %.1fHz): x=%.0f y=%.0f cm", pose_hz_, pkt.x_cm, pkt.y_cm);
  }

  // 到没到飞行起点 A4B8。三道关一起过才算到:TF 是新的、距离进容差、还得稳住
  // launch_stable_s_。只发一次(launch_arrived_sent_),terminal 那边到达是一次性事件。
  void launchArrivedCheck()
  {
    if (launch_arrived_sent_ || !tf_buffer_) { return; }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, laser_link_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      launch_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);   // TF 断了,稳定窗重来
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "拿不到 TF %s->%s,到达检测暂停: %s",
        map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
      return;
    }
    // carto 停更时 buffer 里的旧位姿不会消失,TimePointZero 照样把它取出来 —— 必须自己查时效。
    const rclcpp::Time stamp(tf.header.stamp);
    const double age = (now() - stamp).seconds();
    if (age > launch_tf_fresh_s_) {
      launch_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF 位姿过期 %.2fs(>%.2fs),到达检测暂停(carto 停了?)", age, launch_tf_fresh_s_);
      return;
    }
    // 到起飞点只看 x(行进方向),不看 y:飞车锁 yaw=0 直行修不了横向、y 会漂,
    // 按全距离判会永远到不了。和 route_target_publisher 的地面到达判据保持一致。
    const double dx = tf.transform.translation.x - launch_x_m_;
    const double dist = std::fabs(dx);
    if (dist > launch_tol_m_) {
      launch_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);   // 出容差,稳定窗重来
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "去飞行起点中: 距起飞点 x 方向 %.2fm(容差 %.2fm)", dist, launch_tol_m_);
      return;
    }
    if (launch_stable_since_.nanoseconds() == 0) {
      launch_stable_since_ = now();
      RCLCPP_INFO(get_logger(), "进入 A4B8 容差(%.2fm),等稳定 %.1fs", dist, launch_stable_s_);
      return;
    }
    if ((now() - launch_stable_since_).seconds() < launch_stable_s_) { return; }
    launch_arrived_sent_ = true;
    launch_arrived_id_++;
    launch_arrived_remaining_ = resend_count_;
    RCLCPP_INFO(get_logger(),
      "已到飞行起点 A4B8(距 %.2fm,稳 %.1fs),向车重发 LAUNCH_ARRIVED(FC0D, id=%u)",
      dist, launch_stable_s_, launch_arrived_id_);
  }

  // 飞到写死的难民点 → 发 FC05,车侧 terminal 播报"要投放物资吗"。
  // 跟 launchArrivedCheck 同一套三道关(TF 新鲜 / 进容差 / 稳住),只发一次。
  // 只在空中判(z 由飞控管,这里不看高度):地面段路过那点不该触发。
  void rescueePointCheck()
  {
    if (rescuee_point_sent_ || !tf_buffer_) { return; }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, laser_link_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      rescuee_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }
    const double age = (now() - rclcpp::Time(tf.header.stamp)).seconds();
    if (age > launch_tf_fresh_s_) {
      rescuee_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }
    const double dx = tf.transform.translation.x - rescuee_x_m_;
    const double dy = tf.transform.translation.y - rescuee_y_m_;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > rescuee_tol_m_) {
      rescuee_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }
    if (rescuee_stable_since_.nanoseconds() == 0) {
      rescuee_stable_since_ = now();
      return;
    }
    if ((now() - rescuee_stable_since_).seconds() < rescuee_stable_s_) { return; }
    rescuee_point_sent_ = true;
    rescuee_class_ = static_cast<uint8_t>(rescuee_point_class_);
    rescuee_id_++;
    rescuee_remaining_ = resend_count_;
    RCLCPP_INFO(get_logger(),
      "已到难民点(%.2f,%.2f) 距 %.2fm → 发 FC05(类别=%u,id=%u),等 terminal 确认投放",
      rescuee_x_m_, rescuee_y_m_, dist, rescuee_class_, rescuee_id_);
  }

  void sendTick()
  {
    bool send_airborne = false;
    if (airborne_remaining_ > 0) {
      send_airborne = true;
      airborne_remaining_--;
    } else if (airborne_sent_ && !airborne_acked_) {
      // 首轮 burst 后仍无 ACK：约 1Hz 低频重试，Wi-Fi 恢复后自动闭环。
      airborne_retry_ticks_++;
      if (airborne_retry_ticks_ >= std::max(1, static_cast<int>(send_hz_))) {
        airborne_retry_ticks_ = 0;
        send_airborne = true;
      }
    }
    if (send_airborne) {
      BoolPacket pkt{kMagicAirborne, airborne_id_, 1};
      ::sendto(tx_fd_, &pkt, sizeof(pkt), 0,
        reinterpret_cast<const sockaddr *>(&car_addr_), sizeof(car_addr_));
    }
    if (launch_arrived_remaining_ > 0) {
      BoolPacket pkt{kMagicLaunchArrived, launch_arrived_id_, 1};
      ::sendto(tx_fd_, &pkt, sizeof(pkt), 0,
        reinterpret_cast<const sockaddr *>(&car_addr_), sizeof(car_addr_));
      launch_arrived_remaining_--;
    }
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
    if (status_remaining_ > 0) {
      std::vector<uint8_t> buf(sizeof(StrHeader) + status_payload_.size());
      StrHeader hdr{kMagicStatus, status_id_, static_cast<uint16_t>(status_payload_.size())};
      std::memcpy(buf.data(), &hdr, sizeof(hdr));
      if (!status_payload_.empty()) {
        std::memcpy(buf.data() + sizeof(hdr), status_payload_.data(), status_payload_.size());
      }
      ::sendto(tx_fd_, buf.data(), buf.size(), 0,
        reinterpret_cast<const sockaddr *>(&car_addr_), sizeof(car_addr_));
      status_remaining_--;
    }
  }

  void pollRx()
  {
    uint8_t buf[sizeof(ObstHeader) + kMaxObstFloats * sizeof(float)];
    while (true) {
      const ssize_t n = ::recv(rx_fd_, buf, sizeof(buf), 0);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) { break; }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "recv 失败: %s", std::strerror(errno));
        break;
      }
      if (n < 2) { continue; }
      uint16_t magic;
      std::memcpy(&magic, buf, sizeof(magic));

      // 变长包:巡航航点(float 数组)
      if (magic == kMagicWaypoints && n >= static_cast<ssize_t>(sizeof(ObstHeader))) {
        ObstHeader hdr;
        std::memcpy(&hdr, buf, sizeof(hdr));
        const std::size_t expect =
          sizeof(ObstHeader) + static_cast<std::size_t>(hdr.count) * sizeof(float);
        if (hdr.count > kMaxObstFloats || static_cast<std::size_t>(n) != expect) { continue; }
        // 每次收到合法 FC0A 都回 ACK（包括重复包）。若前一个 ACK 丢了，车板下一次
        // 低频重发会再次得到确认；id 让旧 ACK 不会误停新路线。
        BoolPacket ack{kMagicWaypointsAck, hdr.id, 1};
        ::sendto(tx_fd_, &ack, sizeof(ack), 0,
          reinterpret_cast<const sockaddr *>(&car_addr_), sizeof(car_addr_));
        if (has_wp_id_ && hdr.id == last_wp_id_) { continue; }  // 同一路线只发一次
        has_wp_id_ = true;
        last_wp_id_ = hdr.id;
        std_msgs::msg::Float32MultiArray m;
        m.data.resize(hdr.count);
        std::memcpy(m.data.data(), buf + sizeof(ObstHeader), hdr.count * sizeof(float));
        // terminal 发来的是**场地系**航点 [x_cm,y_cm,z_cm,yaw_deg]×N,而
        // route_test_node/PID 都在 map 系里干活 —— 在这儿换算,下游一律不用管摆放朝向。
        // fieldToMap 用米(要减飞车起点),航点是 cm → /100 进、*100 出;z 不参与;yaw 加旋转量。
        for (std::size_t i = 0; i + 3 < m.data.size(); i += 4) {
          double xm, ym;
          fieldToMap(m.data[i] / 100.0, m.data[i + 1] / 100.0, xm, ym);
          m.data[i]     = static_cast<float>(xm * 100.0);
          m.data[i + 1] = static_cast<float>(ym * 100.0);
          m.data[i + 3] = static_cast<float>(
            normalizeDeg(m.data[i + 3] + field_to_map_deg_));
        }
        // 固定任务的第一批地面路线原来包含“开机位置”作为首航点。坐标转换后它
        // 就是 map 原点 (0,0,0)，路线执行器会先持续追这个无意义目标。地面批次
        // 只需真正的起飞点目标：若首点确实是 map 原点且后面还有点，删掉首点。
        // 空中首点 z>20（原地拉高）绝不能删。
        if (m.data.size() >= 8 && m.data[2] <= 20.0f &&
            std::fabs(m.data[0]) < 1.0f && std::fabs(m.data[1]) < 1.0f) {
          m.data.erase(m.data.begin(), m.data.begin() + 4);
          RCLCPP_INFO(get_logger(), "地面路线删除开机原点，只下发 %zu 个真正目标点",
            m.data.size() / 4);
        }
        wp_pub_->publish(m);
        RCLCPP_INFO(get_logger(), "收到车 UDP waypoints(id=%u,%u float),本地发 /wildlife/waypoints",
          hdr.id, hdr.count);
        continue;
      }

      // 定长包:各类 Bool 信号
      if (n != static_cast<ssize_t>(sizeof(BoolPacket))) { continue; }
      BoolPacket pkt;
      std::memcpy(&pkt, buf, sizeof(pkt));
      if (pkt.magic == kMagicAirborneAck && pkt.value && pkt.id == airborne_id_) {
        if (!airborne_acked_) {
          airborne_acked_ = true;
          airborne_remaining_ = 0;
          RCLCPP_INFO(get_logger(), "车已确认收到 AIRBORNE(FC10,id=%u)，停止重发", pkt.id);
        }
      } else if (pkt.magic == kMagicDone && !done_published_) {
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
      } else if (pkt.magic == kMagicCoverage && !coverage_published_) {
        coverage_published_ = true;
        std_msgs::msg::Float32MultiArray m;
        m.data = {0.0f, 0.0f, 0.0f, 0.0f};  // l_path 忽略内容,仅当开跑信号
        coverage_pub_->publish(m);
        RCLCPP_INFO(get_logger(), "收到车 UDP coverage,本地发 /coverage_area(触发 L 形覆盖)");
      } else if (pkt.magic == kMagicFlightEnable && pkt.value) {
        // 这里按 id 去重(不像 start 那样用一次性 flag):车端 burst 30 包是抗丢包,
        // 语义仍是"一次";但换了 id 就是操作员真的又确认了一次,得放行。
        if (!has_flight_enable_id_ || pkt.id != last_flight_enable_id_) {
          has_flight_enable_id_ = true;
          last_flight_enable_id_ = pkt.id;
          flight_enable_received_ = true;
          airborne_samples_ = 0;
          std_msgs::msg::Bool m; m.data = true;
          flight_enable_pub_->publish(m);
          RCLCPP_INFO(get_logger(),
            "收到车 UDP flight_enable(FC0E, id=%u),本地发 /rescue/flight_search_enable=1(放行起飞)",
            pkt.id);
        }
      }
    }
  }

  std::string car_ip_;
  int to_car_port_, from_car_port_, resend_count_;
  double send_hz_, pose_hz_;
  int airborne_height_cm_{60}, airborne_required_samples_{2};
  // 飞车开机朝向(场地系,度)与由它算出的场地系→map 系旋转量 α 及其 cos/sin
  double start_heading_field_deg_, field_to_map_deg_, cos_a_, sin_a_;
  double field_origin_x_m_{0.0}, field_origin_y_m_{0.0};  // 飞车起点(场地系,米):场地→map 减它
  std::string map_frame_, laser_link_frame_;
  uint16_t pose_id_{0};
  double launch_x_m_, launch_y_m_, launch_tol_m_;
  double launch_stable_s_, launch_tf_fresh_s_, launch_check_hz_;
  double rescuee_x_m_, rescuee_y_m_, rescuee_tol_m_;
  double rescuee_stable_s_, rescuee_check_hz_;
  int rescuee_point_class_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double yolo_conf_thresh_;
  int yolo_debounce_;
  bool yolo_trigger_rescuee_event_{false};

  int tx_fd_{-1}, rx_fd_{-1};
  sockaddr_in car_addr_{};

  uint16_t req_id_{0}, obst_id_{0}, rescuee_id_{0};
  int req_remaining_{0}, obst_remaining_{0}, rescuee_remaining_{0};
  bool flight_enable_received_{false};
  bool airborne_sent_{false};
  int airborne_samples_{0};
  uint16_t airborne_id_{0};
  int airborne_remaining_{0};
  int airborne_retry_ticks_{0};
  bool airborne_acked_{false};
  std::vector<float> obst_payload_;
  bool done_published_{false};
  bool confirm_published_{false};
  bool start_published_{false};
  bool coverage_published_{false};
  // YOLO 去抖 + 当前判定
  uint8_t pending_class_{0}, rescuee_class_{0};
  int hit_count_{0};
  // 状态回传(飞车→车)/ 航点接收(车→飞车)
  std::string status_payload_;
  uint16_t status_id_{0};
  int status_remaining_{0};
  bool has_wp_id_{false};
  uint16_t last_wp_id_{0};
  // Tier0 搜救:放行起飞(收)/ 到达飞行起点(发)
  bool has_flight_enable_id_{false};
  uint16_t last_flight_enable_id_{0};
  bool launch_arrived_sent_{false};
  uint16_t launch_arrived_id_{0};
  int launch_arrived_remaining_{0};
  rclcpp::Time launch_stable_since_{0, 0, RCL_ROS_TIME};
  bool rescuee_point_sent_{false};
  rclcpp::Time rescuee_stable_since_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr req_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obst_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr det_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr confirm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr coverage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flight_enable_pub_;
  rclcpp::TimerBase::SharedPtr send_timer_, rx_timer_, pose_timer_, launch_timer_, rescuee_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XMachineBridge>());
  rclcpp::shutdown();
  return 0;
}
