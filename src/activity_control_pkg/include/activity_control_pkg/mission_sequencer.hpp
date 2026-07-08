#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include "activity_control_pkg/route_target_publisher.hpp"

namespace activity_control_pkg
{

// 灾区两次投放 任务编排状态机(见 docs/relief_drop_mission_design.md)。
//
// 高层顺序:地面直行→右转(-90)弧转投货1→直行到固定起飞点→叫补给车→等补给完成→原地起飞
//           →保持 yaw 飞到难民2 上方→悬停下降投货2→沿机头前进一段→原地垂直降落。
//
// 设计要点:
//  - 全开环、确定性:所有航点写死,阶段推进只靠位置到达(route 队列排空)。
//    摄像头/YOLO 只推视频流到平板,不参与任何门控。
//  - 底层执行沿用 RouteTargetPublisher/chassis_mux,route 指针零改动复用。
//  - 位置/高度门统一用 route 队列是否排空判定(currentIndex()>=size()),
//    不在本节点重复做 TF/height 判定——route 的 isReached 已经算好。
//  - 摄像头舵机(舵机2)角度:地面 120°、飞行 180°,由本节点在相应阶段下发。
//  - 投货/摄像头舵机都经 /servo_cmd([index,angle]) → chassis_bridge 转 $SERVO
//    (与 $VW 共用 ttyS3)。
//  - 补给:到起飞点后发 /resupply_request 叫车;车对接推货2、退开后回 /resupply_done,
//    收到才起飞(桨不提前转)。
class MissionSequencerNode : public rclcpp::Node
{
public:
  explicit MissionSequencerNode(
    std::shared_ptr<RouteTargetPublisherNode> route_node,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class State
  {
    INIT,               // 启动延时 + 摄像头舵机转地面角
    GROUND_TO_DETECT,   // 地面直行到检测点(YOLO 看 rescuee 处),停下
    WAIT_CONFIRM,       // 等 terminal(车)确认(收 /terminal_confirm)才右转投货
    GROUND_TO_DROP1,    // 右转弧转到投货1落点
    DROP_1,             // 舵机1 投货1
    GROUND_TO_TAKEOFF,  // 地面到固定起飞点
    WAIT_RESUPPLY,      // 发 /resupply_request 叫车,停桨等 /resupply_done
    TAKEOFF,            // 摄像头舵机转飞行角 + 切飞行模式 + 原地垂直起飞
    FLY_TO_WP2,         // 飞航点到难民2 上方(全程 yaw 不变)
    DESCEND_2,          // 悬停下降到投放高度(不落地)
    DROP_2,             // 舵机1 投货2
    FORWARD_LAND,       // 沿机头前进一段后原地垂直降落(先平飞再落,不斜切)
    DONE                // 结束
  };

  void enterState(State next);   // 切状态 + 执行进入动作(一次性)
  void tick();                   // 周期检查当前状态出口门
  bool routeDrained() const;     // route 队列排空 = 已到达本段最后航点

  void resupplyCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void confirmCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void missionStartCallback(const std_msgs::msg::Bool::SharedPtr msg);  // terminal 语音"开始救援"启动门

  // 动作辅助
  void sendServo(int index, int angle_deg);
  void addGroundTarget(double x_cm, double y_cm, double yaw_deg);
  void publishResupplyRequest();

  static const char * stateName(State s);

  std::shared_ptr<RouteTargetPublisherNode> route_node_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resupply_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr confirm_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_start_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resupply_req_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr servo_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  State state_ = State::INIT;
  rclcpp::Time state_enter_time_;

  bool resupply_done_ = false;   // 补给握手门
  bool terminal_confirm_ = false;  // terminal 确认门(视觉握手,收到就永久置位)
  bool mission_started_ = false;   // 启动门:收到 terminal"开始救援"信号才离开 INIT
  bool servo_closed_ = false;    // 投货计时:复位是否已发

  // --- 参数(占位待标定,见 docs/relief_drop_mission_design.md §六)---
  bool ground_only_;                // true=只跑地面段,收到补给完成就停在起飞点(不起飞、不进飞行状态)
  bool wait_terminal_confirm_;      // true=到检测点后等 terminal 确认才右转投货(视觉握手);false=直接过
  bool wait_mission_start_;         // true=起来后挂 INIT,等 terminal"开始救援"才开跑;false=延时后自动开跑
  double start_delay_s_;
  double z_ground_cm_;
  double flight_z_cm_;
  double z_drop2_cm_;
  double z_forward_cm_;              // 投货2 后前进平飞的高度(默认沿用 z_drop2,不额外爬升)
  double forward_after_drop2_cm_;    // 投货2 后沿机头前进的距离(cm),到点再垂直降落

  double fwd_x_cm_, fwd_y_cm_, fwd_yaw_deg_;          // 直行前进点(YOLO 看到货处),纯路径整形
  double drop1_x_cm_, drop1_y_cm_, drop1_yaw_deg_;    // 弧转投货1 落点
  double takeoff_x_cm_, takeoff_y_cm_, takeoff_yaw_deg_;  // 固定起飞点(飞行段全程保持此 yaw)
  std::vector<double> fly_waypoints_;                 // 扁平 [x,y,...] cm,最后一个 = 难民2 上方

  // 摄像头舵机(舵机2)角度
  int camera_servo_index_;
  int camera_ground_deg_;   // 地面 120
  int camera_flight_deg_;   // 飞行 180

  // 投货舵机(舵机1)—— 角度占位,待用户给
  int servo_drop_index_;
  int servo_open_deg_;
  int servo_close_deg_;
  double t_drop_s_;
};

}  // namespace activity_control_pkg
