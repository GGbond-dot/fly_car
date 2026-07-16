#!/usr/bin/env python3
"""摄像头舵机跟着地空模式自动切 —— 地面 120°(略朝下),空中 180°(完全朝下)。

拍视频那套是"一条 launch 一个阶段",所以用 servo_set_once 摆一次就够。Tier0 搜救是
**一条路线横跨地面段和空中段**(z=0 跑到起飞点 → 拉高 120cm 巡航),中途必须换角度:
飞行时要垂直俯视才能让 YOLO 认出难民。

触发信号直接用 chassis_mux 已经在发的 /flight_enable —— 它就是"现在算地面还是空中"
的权威判据(按目标 z 是否 >20cm 定),没必要另立一套判断,更不会跟它打架。

servo_cmd = Int16MultiArray data=[index, angle],chassis_bridge 转 $SERVO,index,angle。

用法:  python3 servo_camera_by_mode.py --index 2 --ground-deg 120 --air-deg 180
"""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, Int16MultiArray


class ServoCameraByMode(Node):
    def __init__(self, a):
        super().__init__("servo_camera_by_mode")
        self.a = a
        self.pub = self.create_publisher(Int16MultiArray, "/servo_cmd", 10)
        # chassis_mux 那边是 latched 发的,QoS 要对上,否则收不到;
        # 而且 latched 能让本节点晚起也立刻拿到当前模式。
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub = self.create_subscription(
            Bool, "/flight_enable", self.on_flight_enable, latched)
        self.current = None          # None = 还没定过,收到第一条就发
        self.pending = None
        self.left = 0
        # 单次 publish 可能撞上 chassis_bridge 还没订上,跟 servo_set_once 一样重发几次。
        self.timer = self.create_timer(1.0 / a.rate, self.tick)
        self.get_logger().info(
            f"摄像头舵机{a.index}跟随地空模式: 地面={a.ground_deg}° 空中={a.air_deg}°,"
            f"等 /flight_enable ...")

    def on_flight_enable(self, msg):
        angle = self.a.air_deg if msg.data else self.a.ground_deg
        if angle == self.current:
            return                   # 模式没变就别刷舵机(chassis_mux 是 latched,会重复给)
        self.current = angle
        self.pending = angle
        self.left = max(1, int(self.a.repeat))
        self.get_logger().info(
            f"{'空中' if msg.data else '地面'}态 → 摄像头舵机{self.a.index} = {angle}°")

    def tick(self):
        if self.pending is None or self.left <= 0:
            return
        m = Int16MultiArray()
        m.data = [int(self.a.index), int(self.pending)]
        self.pub.publish(m)
        self.left -= 1
        if self.left <= 0:
            self.pending = None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--index", type=int, default=2, help="摄像头舵机编号(默认 2)")
    p.add_argument("--ground-deg", type=int, default=120, help="地面态角度")
    p.add_argument("--air-deg", type=int, default=180, help="空中态角度(垂直俯视,YOLO 认难民靠它)")
    p.add_argument("--repeat", type=int, default=10, help="每次切换重发几包(抗漏订)")
    p.add_argument("--rate", type=float, default=5.0)
    a, _ = p.parse_known_args()

    rclpy.init()
    node = ServoCameraByMode(a)
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
