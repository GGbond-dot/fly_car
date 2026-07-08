// relief_drop_mission_node — 灾区两次投放 任务编排入口。
//
// 同进程拼两个节点(见 docs/relief_drop_mission_design.md):
//   RouteTargetPublisherNode  航点队列 + 到达推进 + z 编码起降(已有,零改动)
//   MissionSequencerNode       状态机:发段航点、视觉门、补给握手、舵机投货(新增)
//
// 绑一个 SingleThreadedExecutor:sequencer 对队列的写入都走 RouteTargetPublisher
// 的加锁接口,回调串行、时序确定。
//
// 依赖的其它节点(另行启动):diff_drive_controller / chassis_bridge(含 /servo_cmd 转发)
//   / chassis_mux / pid_control_pkg / uart_to_stm32 / yolo_detector / 建图(carto)。

#include <clocale>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "activity_control_pkg/mission_sequencer.hpp"
#include "activity_control_pkg/route_target_publisher.hpp"

int main(int argc, char ** argv)
{
  std::setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);

  auto route = std::make_shared<activity_control_pkg::RouteTargetPublisherNode>();
  auto sequencer = std::make_shared<activity_control_pkg::MissionSequencerNode>(route);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(route);
  exec.add_node(sequencer);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
