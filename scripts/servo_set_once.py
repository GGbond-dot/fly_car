#!/usr/bin/env python3
"""启动时把某舵机设到固定角度 —— 发几秒 /servo_cmd(确保 chassis_bridge 已订阅收到)后退出。

用于每段一开始摆摄像头:地面舵机2=120(略朝下),飞行舵机2=180(完全朝下)。
servo_cmd = Int16MultiArray data=[index, angle],chassis_bridge 转 $SERVO,index,angle。

用法:  python3 servo_set_once.py --index 2 --angle 120
"""
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray


class ServoSetOnce(Node):
    def __init__(self, a):
        super().__init__("servo_set_once")
        self.a = a
        self.pub = self.create_publisher(Int16MultiArray, "/servo_cmd", 10)
        self.n = 0
        self.max = max(1, int(a.duration_s * a.rate))
        self.timer = self.create_timer(1.0 / a.rate, self.tick)
        self.get_logger().info(f"设舵机{a.index} -> {a.angle}度,重发{self.max}次确保 chassis_bridge 收到")

    def tick(self):
        m = Int16MultiArray()
        m.data = [int(self.a.index), int(self.a.angle)]
        self.pub.publish(m)
        self.n += 1
        if self.n >= self.max:
            self.get_logger().info("舵机角度已发,退出。")
            rclpy.shutdown()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--index", type=int, required=True)
    p.add_argument("--angle", type=int, required=True)
    p.add_argument("--duration-s", type=float, default=2.0)
    p.add_argument("--rate", type=float, default=5.0)
    a, _ = p.parse_known_args()

    rclpy.init()
    node = ServoSetOnce(a)
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
