#!/usr/bin/env python3
"""飞行航点自动飞 + 中途投货 —— 持续发"当前航点"到 /target_position 让 pid/飞控追,
到位才推进下一个;到投货航点先悬停(继续发同一点)开货舱投货,投完再飞走;末点低 z 垂直下降。

飞控闭环仍是 pid + uart + carto(架构同 test_flight_square),本节点只负责"发哪个目标点、
何时推进"——角色等同 route_target_publisher,只是多了投货停留。

  /target_position = Float32MultiArray [x_cm, y_cm, z_cm, yaw_deg]  (10Hz 持续发)
  到位判据: xy 用 TF map->laser_link,z 用 /height(Int16 cm,uart 发)
  /servo_cmd = Int16MultiArray [index, angle] -> chassis_bridge 转 $SERVO

改航点/投货点/降落 = 改下面 WAYPOINTS / DROP_INDEX(改完 syncpi 即可,不用 build)。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray, Int16, Int16MultiArray
from tf2_ros import Buffer, TransformListener

# [x_cm, y_cm, z_cm, yaw_deg];yaw 全 0 = 平移飞行,机头不变
WAYPOINTS = [
    (0.0,    0.0,   100.0, 0.0),   # 0 原地升空到 100
    (320.0,  0.0,   100.0, 0.0),   # 1
    (320.0, -117.0, 100.0, 0.0),   # 2
    (132.0, -117.0, 100.0, 0.0),   # 3
    (212.0, -244.0, 100.0, 0.0),   # 4 ← 投货(悬停开舱→复位→再飞走)
    (320.0, -244.0, 100.0, 0.0),   # 5
    (320.0, -244.0,   4.0, 0.0),   # 6 ← 原地垂直下降落地
]
DROP_INDEX = 4

XY_TOL_CM = 15.0        # 平面到位容差
Z_TOL_CM = 12.0         # 高度到位容差(升空/降落判据)
HOLD_FRAMES = 5         # 连续到位帧(10Hz→0.5s)才算稳,防抖
SERVO_INDEX = 1         # 投货舱舵机
OPEN_DEG = 180          # 开舱倒货
CLOSE_DEG = 90          # 复位
OPEN_DWELL_S = 1.5      # 开舱保持(让货落下)
SETTLE_S = 1.0          # 投完再稳一下才飞走


class FlightWaypointDrop(Node):
    def __init__(self):
        super().__init__("flight_waypoint_drop")
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.tgt_pub = self.create_publisher(Float32MultiArray, "/target_position", 10)
        self.servo_pub = self.create_publisher(Int16MultiArray, "/servo_cmd", 10)
        self.height = None
        self.create_subscription(Int16, "/height", self.on_height, 10)
        self.map_frame = "map"
        self.base_frame = "laser_link"
        self.idx = 0
        self.state = "FLY"        # FLY / DROP_OPEN / DROP_CLOSE / DONE
        self.hold = 0
        self.mark = None
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"飞行航点起飞: {len(WAYPOINTS)}个点,投货点 idx={DROP_INDEX},末点垂直下降。"
            f" xy容差{XY_TOL_CM} z容差{Z_TOL_CM}")

    def on_height(self, m):
        self.height = float(m.data)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_wp(self, i):
        x, y, z, yaw = WAYPOINTS[i]
        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(z), float(yaw)]
        self.tgt_pub.publish(msg)

    def send_servo(self, angle):
        m = Int16MultiArray()
        m.data = [SERVO_INDEX, int(angle)]
        self.servo_pub.publish(m)

    def cur_xy(self):
        try:
            t = self.buf.lookup_transform(self.map_frame, self.base_frame, Time())
            return t.transform.translation.x * 100.0, t.transform.translation.y * 100.0
        except Exception:
            return None

    def reached(self, i):
        xy = self.cur_xy()
        if xy is None:
            return False
        x, y, z, _ = WAYPOINTS[i]
        dxy = math.hypot(xy[0] - x, xy[1] - y)
        dz = abs((self.height if self.height is not None else 0.0) - z)
        return dxy <= XY_TOL_CM and dz <= Z_TOL_CM

    def tick(self):
        # 无论什么状态,始终持续发"当前航点"——追踪 / 悬停都靠它,pid 才不会超时停桨
        self.publish_wp(self.idx)

        if self.state == "FLY":
            if self.reached(self.idx):
                self.hold += 1
                if self.hold >= HOLD_FRAMES:
                    self.hold = 0
                    if self.idx == DROP_INDEX:
                        self.get_logger().info(f"到投货点 idx{self.idx},悬停开货舱投货 $SERVO,{SERVO_INDEX},{OPEN_DEG}")
                        self.send_servo(OPEN_DEG)
                        self.mark = self.now()
                        self.state = "DROP_OPEN"
                    elif self.idx >= len(WAYPOINTS) - 1:
                        self.get_logger().info("到达末点(已垂直降落),任务完成,保持发末点。")
                        self.state = "DONE"
                    else:
                        self.idx += 1
                        self.get_logger().info(f"推进到航点 idx{self.idx}: {WAYPOINTS[self.idx][:3]}")
            else:
                self.hold = 0

        elif self.state == "DROP_OPEN":
            # 继续发 drop 航点(悬停不动),开舱保持 OPEN_DWELL_S
            if self.now() - self.mark >= OPEN_DWELL_S:
                self.send_servo(CLOSE_DEG)
                self.mark = self.now()
                self.state = "DROP_CLOSE"
                self.get_logger().info(f"复位货舱 $SERVO,{SERVO_INDEX},{CLOSE_DEG}")

        elif self.state == "DROP_CLOSE":
            if self.now() - self.mark >= SETTLE_S:
                self.idx += 1
                self.state = "FLY"
                self.get_logger().info(f"投货完成,飞走 -> idx{self.idx}: {WAYPOINTS[self.idx][:3]}")
        # DONE: 只持续发末点保持


def main():
    rclpy.init()
    node = FlightWaypointDrop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
