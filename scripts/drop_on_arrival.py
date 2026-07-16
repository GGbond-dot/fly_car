#!/usr/bin/env python3
"""到位投货 —— TF 判飞车到达目标点后,发 /servo_cmd 开货舱丢货、停 t_drop 再复位。

配合 route_test_node 用:route_test 负责走到 drop1,本节点只管"到了就投货"。
到位判据:map->laser_link 的 xy 落在目标 tol_cm 内、连续 hold_frames 帧。投一次即止。

servo 命令 = Int16MultiArray data=[index, angle],chassis_bridge 转 $SERVO,index,angle。

用法:
  python3 drop_on_arrival.py --target-x-cm 20 --target-y-cm -30 --tol-cm 12 \
      --index 1 --open-deg 180 --close-deg 90 --t-drop-s 1.5
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int16MultiArray
from tf2_ros import Buffer, TransformListener


class DropOnArrival(Node):
    def __init__(self, a):
        super().__init__("drop_on_arrival")
        self.a = a
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(Int16MultiArray, "/servo_cmd", 10)
        self.hold = 0
        self.state = "WAIT"          # WAIT -> OPEN -> CLOSE -> DONE
        self.mark = None
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"到位投货待命: 目标({a.target_x_cm},{a.target_y_cm})cm tol={a.tol_cm}cm "
            f"舵机{a.index} 开{a.open_deg}/复位{a.close_deg} 停留{a.t_drop_s}s")

    def cur_xy(self):
        try:
            tf = self.buf.lookup_transform(self.a.map_frame, self.a.base_frame, Time())
            return tf.transform.translation.x * 100.0, tf.transform.translation.y * 100.0
        except Exception:
            return None

    def send(self, angle):
        m = Int16MultiArray()
        m.data = [int(self.a.index), int(angle)]
        self.pub.publish(m)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.state == "WAIT":
            xy = self.cur_xy()
            if xy is None:
                return
            d = math.hypot(xy[0] - self.a.target_x_cm, xy[1] - self.a.target_y_cm)
            if d <= self.a.tol_cm:
                self.hold += 1
                if self.hold >= self.a.hold_frames:
                    self.get_logger().info(f"到位(距目标{d:.1f}cm),开货舱投货 $SERVO,{self.a.index},{self.a.open_deg}")
                    self.send(self.a.open_deg)
                    self.mark = now
                    self.state = "OPEN"
            else:
                self.hold = 0
        elif self.state == "OPEN":
            if now - self.mark >= self.a.t_drop_s:
                self.get_logger().info(f"复位货舱 $SERVO,{self.a.index},{self.a.close_deg}")
                self.send(self.a.close_deg)
                self.mark = now
                self.state = "CLOSE"
        elif self.state == "CLOSE":
            if now - self.mark >= 0.5:
                self.get_logger().info("投货完成。可 Ctrl-C 关掉,飞车原地别动再跑 step3。")
                self.state = "DONE"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--target-x-cm", type=float, required=True)
    p.add_argument("--target-y-cm", type=float, required=True)
    p.add_argument("--tol-cm", type=float, default=12.0)
    p.add_argument("--index", type=int, default=1)
    p.add_argument("--open-deg", type=int, default=180)
    p.add_argument("--close-deg", type=int, default=90)
    p.add_argument("--t-drop-s", type=float, default=1.5)
    p.add_argument("--hold-frames", type=int, default=5)
    p.add_argument("--map-frame", default="map")
    p.add_argument("--base-frame", default="laser_link")
    a, _ = p.parse_known_args()

    rclpy.init()
    node = DropOnArrival(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
