#!/usr/bin/env python3
"""投放中断 —— 巡航中收到"确认投放"就:降到 50cm → 投货 → 升回原高度 → 接着飞。

所谓"中断"是类比:不打断 route_target_publisher 的队列,而是往它前面**插队**几个航点
(/route/insert_waypoints → insertNext),插的做完了原巡航路线自动接着走。

    巡航中(120cm) ──收到 /terminal_confirm──┐
                                            ├→ 插 [当前xy, 50cm] → 到位
                                            ├→ 投货(舵机1 开→停→关)
                                            └→ 插 [当前xy, 原高度] → 到位 → 继续原路线

链路(复用已有的,没新造 magic):
    飞车到点/YOLO → FC05 → 车 → /rescuee_detected → terminal 播报"要投放物资吗"
    你答"需要" → terminal → FC06 → 飞车 /terminal_confirm → **本节点**

⚠ 旧剧本 mission_sequencer 也订 /terminal_confirm。两个别同时起,否则一个信号两处响应。

用法:  python3 rescue_drop_sequencer.py --drop-z 50 --servo-index 1
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, Float32MultiArray, Int16, Int16MultiArray

import tf2_ros

IDLE, DESCENDING, DROPPING, ASCENDING = "IDLE", "DESCENDING", "DROPPING", "ASCENDING"


class RescueDropSequencer(Node):
    def __init__(self, a):
        super().__init__("rescue_drop_sequencer")
        self.a = a
        self.state = IDLE
        self.hold_xy = None        # 进中断时锁住的 xy —— 全程在这一点上下,不许漂
        self.hold_yaw = 0.0
        self.cruise_z = None       # 进中断前的高度,升回时用
        self.t_state = 0.0
        self.has_height = False
        self.current_height_cm = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        # 桥是 latched 发的(FC06 → /terminal_confirm),QoS 要对上
        self.create_subscription(Bool, "/terminal_confirm", self.on_confirm, latched)
        # Cartographer 是 2D，TF 的 z 恒接近 0，绝不能拿它判断是否已降到投放高度。
        # 飞控串口节点发布的 /height 才是真实激光测高。
        self.create_subscription(Int16, "/height", self.on_height, 10)
        self.insert_pub = self.create_publisher(
            Float32MultiArray, "/route/insert_waypoints", 10)
        self.servo_pub = self.create_publisher(Int16MultiArray, "/servo_cmd", 10)
        self.done_pub = self.create_publisher(Bool, "/rescue/drop_done", latched)

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"投放中断待命: 降到 {a.drop_z}cm 投货(舵机{a.servo_index}: "
            f"{a.open_deg}°开 → {a.t_drop_s}s → {a.close_deg}°关) 再升回。等 /terminal_confirm ...")

    # ---------- TF ----------
    def on_height(self, msg):
        self.current_height_cm = float(msg.data)
        self.has_height = True

    def pose(self):
        """TF 取 x/y/yaw，真实 /height 取 z。"""
        if not self.has_height:
            return None
        try:
            tf = self.tf_buffer.lookup_transform("map", "laser_link", rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
        return (t.x * 100.0, t.y * 100.0, self.current_height_cm, yaw)

    def insert(self, x_cm, y_cm, z_cm, yaw_deg):
        m = Float32MultiArray()
        m.data = [float(x_cm), float(y_cm), float(z_cm), float(yaw_deg)]
        self.insert_pub.publish(m)

    def servo(self, angle):
        m = Int16MultiArray()
        m.data = [int(self.a.servo_index), int(angle)]
        for _ in range(5):          # 重发几包,别让 chassis_bridge 漏订
            self.servo_pub.publish(m)

    # ---------- 状态机 ----------
    def on_confirm(self, msg):
        if not msg.data:
            return
        if self.state != IDLE:
            self.get_logger().info(f"投放进行中(state={self.state}),忽略重复确认")
            return
        if not self.has_height:
            self.get_logger().error("还没收到 /height,这次投放取消(高度不明不敢开舱)")
            return
        p = self.pose()
        if p is None:
            self.get_logger().error("拿不到 TF map→laser_link,这次投放取消(位置不明不敢乱降)")
            return
        x, y, z, yaw = p
        if z < self.a.drop_z + self.a.z_tol:
            self.get_logger().warn(
                f"当前高度 {z:.0f}cm 已经不比投放高度 {self.a.drop_z}cm 高,"
                f"不下降,直接投货")
            self.hold_xy, self.hold_yaw, self.cruise_z = (x, y), yaw, z
            self.enter(DROPPING)
            self.servo(self.a.open_deg)
            return
        # 锁住此刻的 xy:整个中断都在这一点上下,免得插队航点用了后来漂掉的位置
        self.hold_xy, self.hold_yaw, self.cruise_z = (x, y), yaw, z
        self.get_logger().info(
            f"确认投放 → 在 ({x:.0f},{y:.0f}) 原地降到 {self.a.drop_z}cm(当前 {z:.0f}cm)")
        self.insert(x, y, self.a.drop_z, yaw)
        self.enter(DESCENDING)

    def enter(self, state):
        self.state = state
        self.t_state = self.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.t_state

    def tick(self):
        if self.state == IDLE:
            return
        p = self.pose()

        if self.state == DESCENDING:
            if self.elapsed() > self.a.timeout_s:
                self.get_logger().error(
                    f"降到 {self.a.drop_z}cm 超时 {self.a.timeout_s}s,放弃投放,升回继续任务")
                self.ascend()
                return
            if p and abs(p[2] - self.a.drop_z) <= self.a.z_tol:
                self.get_logger().info(f"已到 {p[2]:.0f}cm,投货")
                self.servo(self.a.open_deg)
                self.enter(DROPPING)

        elif self.state == DROPPING:
            if self.elapsed() >= self.a.t_drop_s:
                self.servo(self.a.close_deg)
                self.get_logger().info("货已投,舵机复位,升回巡航高度")
                self.ascend()

        elif self.state == ASCENDING:
            if self.elapsed() > self.a.timeout_s:
                self.get_logger().warn("升回超时,当作完成 —— 原巡航路线自己会把它带回高度")
                self.finish()
                return
            if p and abs(p[2] - self.cruise_z) <= self.a.z_tol:
                self.get_logger().info(f"已回到 {p[2]:.0f}cm,退出中断,继续原巡航路线")
                self.finish()

    def ascend(self):
        x, y = self.hold_xy
        self.insert(x, y, self.cruise_z, self.hold_yaw)
        self.enter(ASCENDING)

    def finish(self):
        m = Bool()
        m.data = True
        self.done_pub.publish(m)
        self.state = IDLE
        self.hold_xy = None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--drop-z", type=float, default=50.0, help="投放高度(cm)")
    p.add_argument("--z-tol", type=float, default=8.0, help="高度到位容差(cm)")
    p.add_argument("--servo-index", type=int, default=1, help="货舱舵机编号")
    p.add_argument("--open-deg", type=int, default=180, help="开舱角度")
    p.add_argument("--close-deg", type=int, default=90, help="复位角度")
    p.add_argument("--t-drop-s", type=float, default=1.5, help="开舱保持时长")
    p.add_argument("--timeout-s", type=float, default=20.0, help="升降超时保护")
    a, _ = p.parse_known_args()

    rclpy.init()
    node = RescueDropSequencer(a)
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
