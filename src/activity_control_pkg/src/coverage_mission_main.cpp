// coverage_mission_node — 平地遍历 + 遇障起飞 的任务编排入口。
//
// 同进程拼三个节点(见 docs/coverage_flyover_mission_design.md):
//   RouteTargetPublisherNode  航点队列 + 到达推进 + z 编码起降(已有)
//   CoverageGeneratorNode      弓字形地面航点 → route->addTarget()(新增)
//   ObstacleDecisionNode       遇墙 → route->insertNext() 插越障航点(新增)
//
// 三者绑在一个 SingleThreadedExecutor 里:覆盖/决策对队列的写入都通过
// RouteTargetPublisher 的接口(内部加锁),回调串行,时序确定。

#include <clocale>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "activity_control_pkg/coverage_generator.hpp"
#include "activity_control_pkg/obstacle_decision.hpp"
#include "activity_control_pkg/route_target_publisher.hpp"

int main(int argc, char ** argv)
{
  std::setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);

  auto route = std::make_shared<activity_control_pkg::RouteTargetPublisherNode>();
  auto coverage = std::make_shared<activity_control_pkg::CoverageGeneratorNode>(route);
  auto decision = std::make_shared<activity_control_pkg::ObstacleDecisionNode>(route);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(route);
  exec.add_node(coverage);
  exec.add_node(decision);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
