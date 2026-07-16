#!/usr/bin/env python3
"""极简:让飞车只走一条平行 x 轴的直线(航向锁 0),默认 300cm。没有转弯,纯测直线段。

前提:**另一个终端已经**起着 launch:
    export ROS_DOMAIN_ID=1
    ros2 launch my_launch l_path_tune.launch.py

然后本终端发直线,车就走:
    export ROS_DOMAIN_ID=1
    python3 scripts/run_straight.py               # 走 300cm
    python3 scripts/run_straight.py --dist-cm 200 # 想走多长自己改

放车:车头朝你想要的“正前方”摆正 —— carto 把开机那一刻的朝向定为 yaw=0,
本脚本让车锁 yaw=0 直行,就是“保持初始朝向走直线”。

反复跑:把车搬回起点、重新摆正朝向,再运行一次。
⚠ 车会真跑 —— 拆掉全部桨。
"""

from __future__ import annotations

import argparse
import os
import time


def main() -> int:
    parser = argparse.ArgumentParser(description="让飞车只走一条平行 x 轴的直线。拆桨。")
    parser.add_argument("--dist-cm", type=float, default=300.0, help="直线长度(cm),默认 300")
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

    # 一条直线:(0,0) 航向 0 出发 → (dist,0)。两点 yaw 都是 0 = 全程锁初始朝向,无转弯。
    route = [0.0, 0.0, 0.0, 0.0, float(args.dist_cm), 0.0, 0.0, 0.0]

    print(f"⚠ 拆桨了吗?这就让车沿 +x 直走 {args.dist_cm:.0f}cm。")

    rclpy.init(args=[])
    node = rclpy.create_node("run_straight")

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

        print(f"发直线路线: {route}")
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
            print("⚠ 使能了但 /cmd_vel 没速度 —— 多半车已在起点(直线太短已算到达),"
                  "或控制器还在等 carto TF。")
        else:
            print("✓ 车已开始走直线。脚本退出后车继续走到头。")
        print("再走一趟:搬回起点、摆正朝向,重新 `python3 scripts/run_straight.py`。")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
