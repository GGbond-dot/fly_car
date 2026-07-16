#include "activity_control_pkg/mission_sequencer.hpp"

#include <cmath>
#include <stdexcept>

namespace activity_control_pkg
{

MissionSequencerNode::MissionSequencerNode(
  std::shared_ptr<RouteTargetPublisherNode> route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("mission_sequencer", options),
  route_node_(std::move(route_node))
{
  ground_only_   = declare_parameter("ground_only", true);   // 当前阶段:只跑地面,到起飞点即停(不飞)
  wait_terminal_confirm_ = declare_parameter("wait_terminal_confirm", true);  // 到检测点等 terminal 确认才右转投货
  wait_mission_start_ = declare_parameter("wait_mission_start", true);        // 起来后等 terminal"开始救援"才开跑
  start_delay_s_ = declare_parameter("start_delay_s", 2.0);
  z_ground_cm_   = declare_parameter("z_ground_cm", 4.0);
  flight_z_cm_   = declare_parameter("flight_z_cm", 100.0);   // 原地起飞/巡航高度
  z_drop2_cm_    = declare_parameter("z_drop2_cm", 50.0);     // 难民2 悬停投放高度(空中,不落地)
  z_forward_cm_  = declare_parameter("z_forward_cm", 50.0);   // 投货2 后前进平飞高度(默认=z_drop2)
  forward_after_drop2_cm_ = declare_parameter("forward_after_drop2_cm", 100.0);  // 投货2 后沿机头前进距离

  // 写死的位置(map 坐标,两车建图起点固定)—— 现场标定值,右转=yaw -90
  fwd_x_cm_       = declare_parameter("fwd_x_cm", 245.0);    // 直行前进点(YOLO 看到货处),纯路径整形
  fwd_y_cm_       = declare_parameter("fwd_y_cm", 0.0);
  fwd_yaw_deg_    = declare_parameter("fwd_yaw_deg", 0.0);
  drop1_x_cm_     = declare_parameter("drop1_x_cm", 265.0);  // 右转(-90)走弧到此,投货1
  drop1_y_cm_     = declare_parameter("drop1_y_cm", -30.0);
  drop1_yaw_deg_  = declare_parameter("drop1_yaw_deg", -90.0);
  takeoff_x_cm_   = declare_parameter("takeoff_x_cm", 265.0);  // 继续直行(-y)到起飞点
  takeoff_y_cm_   = declare_parameter("takeoff_y_cm", -97.0);
  takeoff_yaw_deg_= declare_parameter("takeoff_yaw_deg", -90.0);  // 飞行段全程保持此 yaw
  // 飞行航点扁平 [x,y,...] cm,最后一个 = 难民2 上方(投货2 下降点)。
  fly_waypoints_  = declare_parameter<std::vector<double>>(
    "fly_waypoints", std::vector<double>{-8.0, -270.0});

  camera_servo_index_ = declare_parameter("camera_servo_index", 2);
  camera_ground_deg_  = declare_parameter("camera_ground_deg", 120);
  camera_flight_deg_  = declare_parameter("camera_flight_deg", 180);

  servo_drop_index_ = declare_parameter("servo_drop_index", 1);
  servo_open_deg_   = declare_parameter("servo_open_deg", 180);   // 倒货(实测)
  servo_close_deg_  = declare_parameter("servo_close_deg", 90);   // 初始/复位(实测)
  t_drop_s_         = declare_parameter("t_drop_s", 1.5);

  const std::string servo_cmd_topic    = declare_parameter("servo_cmd_topic", "/servo_cmd");
  const std::string resupply_topic     = declare_parameter("resupply_topic", "/resupply_done");
  const std::string resupply_req_topic = declare_parameter("resupply_request_topic", "/resupply_request");
  const std::string confirm_topic      = declare_parameter("terminal_confirm_topic", "/terminal_confirm");
  const std::string mission_start_topic = declare_parameter("mission_start_topic", "/mission_start");

  if (fly_waypoints_.size() < 2 || fly_waypoints_.size() % 2 != 0) {
    RCLCPP_FATAL(get_logger(),
      "fly_waypoints 必须是 2 的倍数 [x,y,...],当前 %zu 个", fly_waypoints_.size());
    throw std::runtime_error("invalid fly_waypoints parameter");
  }

  // 订阅端用 volatile+reliable:terminal(py-xiaozhi rclpy 默认 volatile)与 xmachine_bridge(latched)
  // 两种发布端都能收(volatile 订阅兼容 volatile/transient_local 发布);节点常驻先于按键,不需 latched 补发。
  resupply_sub_ = create_subscription<std_msgs::msg::Bool>(
    resupply_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&MissionSequencerNode::resupplyCallback, this, std::placeholders::_1));
  // terminal 确认(经 xmachine_bridge 从车 UDP 转来),视觉握手门
  confirm_sub_ = create_subscription<std_msgs::msg::Bool>(
    confirm_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&MissionSequencerNode::confirmCallback, this, std::placeholders::_1));
  // terminal 语音"开始救援"(经 xmachine_bridge 从车 UDP 转来),启动门
  mission_start_sub_ = create_subscription<std_msgs::msg::Bool>(
    mission_start_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&MissionSequencerNode::missionStartCallback, this, std::placeholders::_1));
  // 叫车信号 latched(边沿使能),车侧后启动也能收到
  resupply_req_pub_ = create_publisher<std_msgs::msg::Bool>(
    resupply_req_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  servo_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(servo_cmd_topic, rclcpp::QoS(10));

  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&MissionSequencerNode::tick, this));

  state_enter_time_ = now();
  RCLCPP_INFO(get_logger(),
    "mission_sequencer 启动:%s。飞行航点 %zu 个,巡航 z=%.0fcm。",
    wait_mission_start_ ? "挂 INIT 等 terminal'开始救援'(/mission_start)才开跑"
                        : "延时后自动开跑",
    fly_waypoints_.size() / 2, flight_z_cm_);
}

const char * MissionSequencerNode::stateName(State s)
{
  switch (s) {
    case State::INIT: return "INIT";
    case State::GROUND_TO_DETECT: return "GROUND_TO_DETECT";
    case State::WAIT_CONFIRM: return "WAIT_CONFIRM";
    case State::GROUND_TO_DROP1: return "GROUND_TO_DROP1";
    case State::DROP_1: return "DROP_1";
    case State::GROUND_TO_TAKEOFF: return "GROUND_TO_TAKEOFF";
    case State::WAIT_RESUPPLY: return "WAIT_RESUPPLY";
    case State::TAKEOFF: return "TAKEOFF";
    case State::FLY_TO_WP2: return "FLY_TO_WP2";
    case State::DESCEND_2: return "DESCEND_2";
    case State::DROP_2: return "DROP_2";
    case State::FORWARD_LAND: return "FORWARD_LAND";
    case State::DONE: return "DONE";
  }
  return "?";
}

bool MissionSequencerNode::routeDrained() const
{
  // 队列排空:当前索引推进到末尾(所有已加航点都到达)。空队列(size==0)不算到达。
  return route_node_->size() > 0 && route_node_->currentIndex() >= route_node_->size();
}

void MissionSequencerNode::sendServo(int index, int angle_deg)
{
  std_msgs::msg::Int16MultiArray msg;
  msg.data = {static_cast<int16_t>(index), static_cast<int16_t>(angle_deg)};
  servo_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "舵机: $SERVO,%d,%d", index, angle_deg);
}

void MissionSequencerNode::addGroundTarget(double x_cm, double y_cm, double yaw_deg)
{
  // 地面段:flight_mode 关,z 用地面标称值,yaw 生效(地面到达判定看 yaw)。
  route_node_->addTarget(Target{x_cm, y_cm, z_ground_cm_, yaw_deg, false});
}

void MissionSequencerNode::publishResupplyRequest()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  resupply_req_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "发 /resupply_request:飞车已到起飞点,叫补给车过来。");
}

void MissionSequencerNode::enterState(State next)
{
  state_ = next;
  state_enter_time_ = now();
  servo_closed_ = false;
  RCLCPP_INFO(get_logger(), "→ 进入状态 %s", stateName(next));

  switch (next) {
    case State::GROUND_TO_DETECT:
      sendServo(camera_servo_index_, camera_ground_deg_);   // 摄像头转地面角(120),对准 rescuee
      sendServo(servo_drop_index_, servo_close_deg_);        // 倒货舵机先复位到初始(90),关好箱子再走
      addGroundTarget(fwd_x_cm_, fwd_y_cm_, fwd_yaw_deg_);   // 直行到检测点(245,0)停下
      break;

    case State::WAIT_CONFIRM:
      RCLCPP_INFO(get_logger(), "到检测点,等 terminal 确认(/terminal_confirm)才右转投货。");
      break;

    case State::GROUND_TO_DROP1:
      addGroundTarget(drop1_x_cm_, drop1_y_cm_, drop1_yaw_deg_);  // 右转弧转到投货1落点
      break;

    case State::DROP_1:
      sendServo(servo_drop_index_, servo_open_deg_);   // 开投放口,t_drop 后复位(见 tick)
      break;

    case State::GROUND_TO_TAKEOFF:
      addGroundTarget(takeoff_x_cm_, takeoff_y_cm_, takeoff_yaw_deg_);
      break;

    case State::WAIT_RESUPPLY:
      resupply_done_ = false;         // 只认进入本态之后收到的完成信号
      publishResupplyRequest();       // 叫车过来对接推货2
      break;

    case State::TAKEOFF:
      sendServo(camera_servo_index_, camera_flight_deg_);   // 摄像头转飞行角(180)
      // 切飞行模式(全局 z 顶成 flight_z),在起飞点原地垂直拔高
      route_node_->setFlightMode(true, flight_z_cm_);
      route_node_->addTarget(Target{takeoff_x_cm_, takeoff_y_cm_, flight_z_cm_, takeoff_yaw_deg_, false});
      break;

    case State::FLY_TO_WP2:
      // 逐个压入飞行航点(flight_mode 开 → z 全顶成 flight_z)。yaw 全程用起飞时的 yaw,
      // 保持机头朝向不变(空中到达判定忽略 yaw,但发布的 yaw 是飞控要保持的朝向)。
      for (std::size_t i = 0; i + 1 < fly_waypoints_.size(); i += 2) {
        route_node_->addTarget(
          Target{fly_waypoints_[i], fly_waypoints_[i + 1], flight_z_cm_, takeoff_yaw_deg_, false});
      }
      break;

    case State::DESCEND_2: {
      // 悬停下降:压低全局覆盖高度到 z_drop2,原 xy 悬停下降(不落地),yaw 不变
      route_node_->setFlightMode(true, z_drop2_cm_);
      const std::size_t n = fly_waypoints_.size();
      route_node_->addTarget(
        Target{fly_waypoints_[n - 2], fly_waypoints_[n - 1], z_drop2_cm_, takeoff_yaw_deg_, false});
      break;
    }

    case State::DROP_2:
      sendServo(servo_drop_index_, servo_open_deg_);
      break;

    case State::FORWARD_LAND: {
      // 先沿机头(takeoff_yaw)平飞 forward_after_drop2 距离,到点再原地垂直降落(land_after)。
      // z 保持 z_forward(默认=z_drop2,不额外爬升);land_after 到点后 route 自动退飞行模式垂直下落。
      route_node_->setFlightMode(true, z_forward_cm_);
      const std::size_t n = fly_waypoints_.size();
      const double yaw_rad = takeoff_yaw_deg_ * M_PI / 180.0;
      const double fx = fly_waypoints_[n - 2] + std::cos(yaw_rad) * forward_after_drop2_cm_;
      const double fy = fly_waypoints_[n - 1] + std::sin(yaw_rad) * forward_after_drop2_cm_;
      route_node_->addTarget(Target{fx, fy, z_forward_cm_, takeoff_yaw_deg_, /*land_after=*/true});
      break;
    }

    case State::DONE:
      RCLCPP_INFO(get_logger(), "任务结束。");
      break;

    case State::INIT:
      break;
  }
}

void MissionSequencerNode::tick()
{
  const double elapsed = (now() - state_enter_time_).seconds();

  switch (state_) {
    case State::INIT:
      // 启动门:wait_mission_start=true 时挂着等 terminal"开始救援"(mission_started_);
      // false 时沿用旧行为(延时后自动开跑)。start_delay 作起跑前的短暂 settle。
      if ((mission_started_ || !wait_mission_start_) && elapsed >= start_delay_s_) {
        enterState(State::GROUND_TO_DETECT);
      }
      break;

    case State::GROUND_TO_DETECT:
      if (routeDrained()) {
        enterState(wait_terminal_confirm_ ? State::WAIT_CONFIRM : State::GROUND_TO_DROP1);
      }
      break;

    case State::WAIT_CONFIRM:
      if (terminal_confirm_) { enterState(State::GROUND_TO_DROP1); }
      break;

    case State::GROUND_TO_DROP1:
      if (routeDrained()) { enterState(State::DROP_1); }
      break;

    case State::DROP_1:
      if (!servo_closed_ && elapsed >= t_drop_s_) {
        sendServo(servo_drop_index_, servo_close_deg_);
        servo_closed_ = true;
        enterState(State::GROUND_TO_TAKEOFF);
      }
      break;

    case State::GROUND_TO_TAKEOFF:
      if (routeDrained()) { enterState(State::WAIT_RESUPPLY); }
      break;

    case State::WAIT_RESUPPLY:
      if (resupply_done_) {
        if (ground_only_) {
          RCLCPP_INFO(get_logger(), "ground_only:补给完成,地面段结束,停在起飞点(不起飞)。");
          enterState(State::DONE);
        } else {
          enterState(State::TAKEOFF);
        }
      }
      break;

    case State::TAKEOFF:
      if (routeDrained()) { enterState(State::FLY_TO_WP2); }  // 拔高到 flight_z 到位
      break;

    case State::FLY_TO_WP2:
      if (routeDrained()) { enterState(State::DESCEND_2); }
      break;

    case State::DESCEND_2:
      if (routeDrained()) { enterState(State::DROP_2); }  // 降到 z_drop2 到位
      break;

    case State::DROP_2:
      if (!servo_closed_ && elapsed >= t_drop_s_) {
        sendServo(servo_drop_index_, servo_close_deg_);
        servo_closed_ = true;
        enterState(State::FORWARD_LAND);
      }
      break;

    case State::FORWARD_LAND:
      // 前进点带 land_after:route 到点后自插垂直下降航点,降到地面队列才排空。
      if (routeDrained()) { enterState(State::DONE); }
      break;

    case State::DONE:
      break;
  }
}

void MissionSequencerNode::resupplyCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resupply_done_ = true;
    RCLCPP_INFO(get_logger(), "收到 /resupply_done=1:补给完成、车已退开,可起飞。");
  }
}

void MissionSequencerNode::confirmCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 视觉握手门:收到就永久置位(不复位)。即便 terminal 早于飞车到检测点确认,
  // 飞车仍会先走到检测点(fwd)再检查此门,不会提前右转,顺序有保证。
  if (msg->data) {
    terminal_confirm_ = true;
    RCLCPP_INFO(get_logger(), "收到 /terminal_confirm=1:terminal 放行,右转投货。");
  }
}

void MissionSequencerNode::missionStartCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 启动门:收到就永久置位(不复位)。只在 INIT 态起作用,任务已开跑后再收到无影响。
  if (msg->data && !mission_started_) {
    mission_started_ = true;
    RCLCPP_INFO(get_logger(), "收到 /mission_start=1:terminal 发起'开始救援',离开 INIT 开跑。");
  }
}

}  // namespace activity_control_pkg
