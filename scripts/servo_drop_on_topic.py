#!/usr/bin/env python3
"""话题触发投货 —— 收到 /drop_now(Bool true)就开货舱→停 t_drop→复位(投一次,可重复)。

手飞到难民上方时,发一次话题即投第二次货,不用上位机算航点。
servo_cmd = Int16MultiArray data=[index, angle],chassis_bridge 转 $SERVO。

launch 里起:  python3 servo_drop_on_topic.py --index 1 --open-deg 180 --close-deg 90 --t-drop-s 1.5
手动触发:     ros2 topic pub --once /drop_now std_msgs/msg/Bool "{data: true}"
             (飞车在域1,发之前先 export ROS_DOMAIN_ID=1)
"""
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16MultiArray


class ServoDropOnTopic(Node):
    def __init__(self, a):
        super().__init__("servo_drop_on_topic")
        self.a = a
        self.pub = self.create_publisher(Int16MultiArray, "/servo_cmd", 10)
        self.sub = self.create_subscription(Bool, "/drop_now", self.on_trig, 10)
        self.state = "IDLE"          # IDLE -> OPEN -> CLOSE -> IDLE
        self.mark = None
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f"投货待命: 发 /drop_now=true 触发。舵机{a.index} 开{a.open_deg}/复位{a.close_deg} 停留{a.t_drop_s}s")

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def send(self, angle):
        m = Int16MultiArray()
        m.data = [int(self.a.index), int(angle)]
        self.pub.publish(m)

    def on_trig(self, msg):
        if msg.data and self.state == "IDLE":
            self.get_logger().info(f"收到 /drop_now,开货舱投货 $SERVO,{self.a.index},{self.a.open_deg}")
            self.send(self.a.open_deg)
            self.mark = self.now()
            self.state = "OPEN"

    def tick(self):
        if self.state == "OPEN" and self.now() - self.mark >= self.a.t_drop_s:
            self.get_logger().info(f"复位货舱 $SERVO,{self.a.index},{self.a.close_deg}")
            self.send(self.a.close_deg)
            self.mark = self.now()
            self.state = "CLOSE"
        elif self.state == "CLOSE" and self.now() - self.mark >= 0.5:
            self.get_logger().info("投货完成,可再次 /drop_now。")
            self.state = "IDLE"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--index", type=int, default=1)
    p.add_argument("--open-deg", type=int, default=180)
    p.add_argument("--close-deg", type=int, default=90)
    p.add_argument("--t-drop-s", type=float, default=1.5)
    a, _ = p.parse_known_args()

    rclpy.init()
    node = ServoDropOnTopic(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
