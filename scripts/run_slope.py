#!/usr/bin/env python3
"""极简:车头保持朝 x 轴(yaw=0)起步,边走边顺时针带一点转,走出一条斜线。

不含原地转 —— 起步时车头朝 0、目标航向是 -deg,差值在 align_gate 内,控制器直接
进“边走边转”:同时给前进速度和转向,走弧线过渡到 -deg 再沿直线走。原地不快转 →
不激 carto。

默认:向右(顺时针)16°、斜线长 300cm。方向按你现在的机头=map 的 +x 轴、斜线在右边。

前提:**另一个终端已经**起着 launch:
    export ROS_DOMAIN_ID=1
    ros2 launch my_launch l_path_tune.launch.py

本终端:
    export ROS_DOMAIN_ID=1
    python3 scripts/run_slope.py                 # 向右16°、走300cm
    python3 scripts/run_slope.py --deg 16 --dist-cm 364   # 想改角度/长度
    python3 scripts/run_slope.py --deg -16       # 负数=向左(逆时针)

放车:车头朝你的“正前方”摆正(carto 把开机朝向定为 yaw=0)。
⚠ 车会真跑 —— 拆掉全部桨。
"""

from __future__ import annotations

import argparse
import math
import os
import time


def main() -> int:
    parser = argparse.ArgumentParser(description="车头朝x轴起步,边走边带一点转走斜线。拆桨。")
    parser.add_argument("--deg", type=float, default=16.0,
                        help="顺时针(向右)转多少度,默认16;负数=向左")
    parser.add_argument("--dist-cm", type=float, default=300.0, help="斜线长度(cm),默认300")
    args = parser.parse_args()
    if args.dist_cm <= 0:
        print("error: --dist-cm 要正数")
        return 2

    if os.environ.get("ROS_DOMAIN_ID") != "1":
        print("error: ROS_DOMAIN_ID 必须=1  →  export ROS_DOMAIN_ID=1")
        return 2

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float32MultiArray

    # 顺时针(向右)= yaw 为负。终点沿航向 -deg、距离 dist 处。
    heading_deg = -args.deg
    hr = math.radians(heading_deg)
    end_x = args.dist_cm * math.cos(hr)
    end_y = args.dist_cm * math.sin(hr)

    # 只发一个航点=终点,航向=heading_deg。控制器锁这个航向,起步就边走边转(不原地转)。
    route = [round(end_x, 1), round(end_y, 1), 0.0, round(heading_deg, 1)]

    print(f"⚠ 拆桨了吗?车头朝x轴起步,边走边{'右' if args.deg >= 0 else '左'}转{abs(args.deg):.0f}°,"
          f"走到 ({route[0]:.0f},{route[1]:.0f})cm、航向{heading_deg:.0f}°。")

    rclpy.init(args=[])
    node = rclpy.create_node("run_slope")

    latched = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    route_pub = node.create_publisher(Float32MultiArray, "/wildlife/waypoints", latched)

    state = {"ground_enable": None, "moving": False}

    def _ground_cb(msg: object) -> None:
        state["ground_enable"] = bool(msg.data)

    def _cmd_cb(msg: object) -> None:
        if abs(float(msg.linear.x)) > 1e-3 or abs(float(msg.angular.z)) > 1e-3:
            state["moving"] = True

    node.create_subscription(Bool, "/ground_enable", _ground_cb, latched)
    node.create_subscription(Twist, "/cmd_vel", _cmd_cb, 10)

    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and route_pub.get_subscription_count() < 1:
            rclpy.spin_once(node, timeout_sec=0.1)
        if route_pub.get_subscription_count() < 1:
            print("error: /wildlife/waypoints 没订阅者 —— 另一个终端的 "
                  "`ros2 launch my_launch l_path_tune.launch.py` 起了吗?")
            return 1

        print(f"发斜线路线(单航点): {route}")
        msg = Float32MultiArray()
        msg.data = list(route)
        for _ in range(3):
            route_pub.publish(msg)
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.02)

        t0 = time.monotonic()
        while time.monotonic() - t0 < 4.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        print(f"ground_enable={state['ground_enable']}  已发速度(车在动)={state['moving']}")
        if state["ground_enable"] is not True:
            print("⚠ ground_enable 不是 true —— chassis_mux 没切地面态,查目标 z / mux。")
        elif not state["moving"]:
            print("⚠ 使能了但 /cmd_vel 没速度 —— 控制器还在等 carto TF?")
        else:
            print("✓ 车已起步,边走边转。脚本退出后车继续走完斜线。")
        print("再跑:搬回起点、车头摆正朝正前方,重新运行即可。")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
