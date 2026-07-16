#!/usr/bin/env python3
"""开环直行 —— 定频发 /cmd_vel(linear.x=speed)走固定距离,不用定位/pure-pursuit。

直线开环比闭环算法准(转弯才需要算法)。走完连发 1s 的 0 速停车再退出;
chassis_bridge 本身 500ms 无 cmd_vel 也会 $STOP,双保险。

用法(飞车板,先起 chassis_bridge):
  python3 open_loop_drive.py --distance-cm 245 --speed 0.16
"""
import argparse

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class OpenLoopDrive(Node):
    def __init__(self, a):
        super().__init__("open_loop_drive")
        self.a = a
        self.pub = self.create_publisher(Twist, a.topic, 10)
        self.duration = (a.distance_cm / 100.0 / a.speed) if a.speed > 0 else 0.0
        self.start = None
        self.stop_frames = 0
        self.timer = self.create_timer(1.0 / a.rate, self.tick)
        self.get_logger().info(
            f"开环直行: v={a.speed}m/s 距离={a.distance_cm}cm 预计{self.duration:.1f}s -> {a.topic}")

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.start is None:
            self.start = now
        elapsed = now - self.start
        t = Twist()
        if elapsed < self.duration:
            t.linear.x = float(self.a.speed)
            self.pub.publish(t)
        else:
            t.linear.x = 0.0
            self.pub.publish(t)
            self.stop_frames += 1
            if self.stop_frames == 1:
                self.get_logger().info(f"到达({elapsed:.1f}s),停车。")
            if self.stop_frames >= int(self.a.rate):   # 发 ~1s 的 0 速
                self.get_logger().info("完成,可 Ctrl-C 关掉,飞车原地别动再跑 step2。")
                rclpy.shutdown()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--speed", type=float, default=0.16, help="线速度 m/s")
    p.add_argument("--distance-cm", type=float, required=True, help="直行距离 cm")
    p.add_argument("--rate", type=float, default=20.0, help="发布频率 Hz")
    p.add_argument("--topic", default="/cmd_vel")
    a, _ = p.parse_known_args()

    rclpy.init()
    node = OpenLoopDrive(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.destroy_node()
    except Exception:
        pass


if __name__ == "__main__":
    main()
