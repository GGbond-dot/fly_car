#!/usr/bin/env python3
"""极简:给飞车发那条固定 L 路线,让车真跑起来。不采集、不分析、不设参数。

前提:**另一个终端已经**起着 launch(carto + 底盘 + route_test_node 在等路线):
    export ROS_DOMAIN_ID=1
    ros2 launch my_launch l_path_tune.launch.py

然后本终端一条命令发 L,车就跑:
    export ROS_DOMAIN_ID=1
    python3 scripts/run_l.py

反复跑:把车**搬回起点**,再运行一次即可(锁航向直行不会自己掉头回原点)。
⚠ 车会真跑 —— 拆掉全部桨。
"""

from __future__ import annotations

import os
import time

# 固定 L:(0,0)→(50,0) 直走,原地转到 74.1°,→(150,350)。与 l_path_tuning.py 同一条。
# 每 4 个一组 [x_cm, y_cm, z_cm, yaw_deg]。yaw 是“从该点出发那一段”的航向。
ROUTE = [
    0.0,   0.0,   0.0, 0.0,
    50.0,  0.0,   0.0, 74.1,
    150.0, 350.0, 0.0, 74.1,
]


def main() -> int:
    if os.environ.get("ROS_DOMAIN_ID") != "1":
        print("error: ROS_DOMAIN_ID 必须=1  →  export ROS_DOMAIN_ID=1")
        return 2

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float32MultiArray

    print("⚠ 拆桨了吗?这就让车真跑。")

    rclpy.init(args=[])
    node = rclpy.create_node("run_l")

    # 与 route_test_node 的订阅对齐:latched(晚起也能拿到最后一条路线)。
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
        # 等 route_test_node 订上,否则路线没人收。
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and route_pub.get_subscription_count() < 1:
            rclpy.spin_once(node, timeout_sec=0.1)
        if route_pub.get_subscription_count() < 1:
            print("error: /wildlife/waypoints 没订阅者 —— 另一个终端的 "
                  "`ros2 launch my_launch l_path_tune.launch.py` 起了吗?")
            return 1

        print(f"发 L 路线: {ROUTE}")
        msg = Float32MultiArray()
        msg.data = [float(v) for v in ROUTE]
        for _ in range(3):  # 发几次确保送达
            route_pub.publish(msg)
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.02)

        # 反馈 4 秒:使能进没进地面态、有没有真发速度 —— 好判断“动没动”。
        t0 = time.monotonic()
        while time.monotonic() - t0 < 4.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        print(f"ground_enable={state['ground_enable']}  已发速度(车在动)={state['moving']}")
        if state["ground_enable"] is not True:
            print("⚠ ground_enable 不是 true —— chassis_mux 没切地面态,查目标 z / mux。")
        elif not state["moving"]:
            print("⚠ 使能了但 /cmd_vel 没速度 —— 多半车已在起点(到点只原地转),"
                  "或控制器还在等 carto TF。把车挪离起点再发一次看看。")
        else:
            print("✓ 车已开始跑 L。脚本退出后车继续跑(route_test_node 心跳维持目标)。")
        print("再跑一趟:把车搬回起点,重新 `python3 scripts/run_l.py`。")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
