#!/usr/bin/env python3
"""四轮底盘健康监控 —— 订 /chassis/wheel_rpm + /cmd_vel,判"哪个轮不对劲"。

为什么四驱需要这个节点(两驱时不需要):
  两驱是"前驱两轮 + 后万向轮",后轮自由,任何一个驱动轮出问题都直接反映在车的
  运动上,看轨迹就知道。四驱不同 —— 四个轮刚性同底盘,某个轮虚接触/打滑时,
  另外三个轮会把车照常推走,**车照跑,只是跑偏**。这时候光看轨迹分不清是
  "控制没调好"还是"某个轮没着地",调参就会调错方向。本节点就是把这件事分开。

四项判据(全部只用相对量,不依赖 rpm 绝对精度):
  1. 同侧失配  同侧两轮机械上必须同速(同一底盘、同侧着地)。|fl-rl| 或 |fr-rr|
               超阈值 = 该侧有一个轮在打滑或没着地。
  2. 单轮停转  命令非零、同侧另一轮在转,而本轮 ~0 = 堵转/断线/编码器挂了。
  3. 命令零但在转  cmd_vel=0 却测到转动 = 被推、溜坡,或底盘没收到 $STOP。
  4. 直行左右偏  w≈0 时左右两侧平均 rpm 之差 = 跑偏的**前馈量**,可直接换算成
               diff_drive_controller 的 w_bias_rps 初值,省一次开环标定。

⚠ 阈值单位是 rpm,但编码器 ppr=255 @50Hz 上报时量化步进约 11.8rpm(巡航 66rpm 时
  ±18%)。所以阈值不能设小,默认按 2 个量化步进给;判据也全部做了持续时间确认
  (hold_s),避免单帧量化跳变误报。这个节点是**诊断**,不参与控制回路。

输出 /chassis/wheel_diag (std_msgs/String):
  正常     "OK"
  异常     "SIDE_MISMATCH:left" / "WHEEL_STALL:rl" / "MOVING_WHILE_IDLE" / ...
  多项异常用 ";" 连接。同时按节流打 ROS 日志。
"""

import sys

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32MultiArray, String
except ImportError:
    print(
        "ERROR: 需要在 ROS2 环境中运行,请先 source /opt/ros/<distro>/setup.bash",
        file=sys.stderr,
    )
    raise


WHEEL_RPM_TOPIC = "/chassis/wheel_rpm"
WHEEL_DIAG_TOPIC = "/chassis/wheel_diag"
CMD_VEL_TOPIC = "cmd_vel"

# 轮序与 chassis_bridge 的 $RPM4 一致:左前/右前/左后/右后
FL, FR, RL, RR = 0, 1, 2, 3
WHEEL_NAMES = ("fl", "fr", "rl", "rr")

DIAG_OK = "OK"
LOG_THROTTLE_S = 2.0


class WheelHealthMonitor(Node):
    def __init__(self):
        super().__init__("wheel_health_monitor")

        # 同侧两轮 rpm 差阈值。默认 24 ≈ 2 个量化步进(ppr=255 @50Hz),低于此判不出来
        self.declare_parameter("side_mismatch_rpm", 24.0)
        # 判"这个轮停了"的 rpm 阈值(含量化死区)
        self.declare_parameter("stall_rpm", 6.0)
        # 认为"底盘应该在动"的最小命令速度
        self.declare_parameter("moving_cmd_mps", 0.05)
        # 判"直行"的最大命令角速度(用于左右偏判据)
        self.declare_parameter("straight_cmd_rps", 0.05)
        # 异常必须持续这么久才上报,滤掉单帧量化跳变
        self.declare_parameter("hold_s", 0.4)
        # cmd_vel 断流多久后不再做与命令相关的判据
        self.declare_parameter("cmd_timeout_s", 0.5)

        self.side_mismatch_rpm = self.get_parameter("side_mismatch_rpm").value
        self.stall_rpm = self.get_parameter("stall_rpm").value
        self.moving_cmd_mps = self.get_parameter("moving_cmd_mps").value
        self.straight_cmd_rps = self.get_parameter("straight_cmd_rps").value
        self.hold_s = self.get_parameter("hold_s").value
        self.cmd_timeout_s = self.get_parameter("cmd_timeout_s").value

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = None
        # 每个故障码 -> 首次出现的时刻;持续超过 hold_s 才真正上报
        self.fault_since = {}
        self.last_reported = None

        self.create_subscription(
            Float32MultiArray, WHEEL_RPM_TOPIC, self.on_wheel_rpm, 10)
        self.create_subscription(Twist, CMD_VEL_TOPIC, self.on_cmd_vel, 10)
        self.diag_publisher = self.create_publisher(String, WHEEL_DIAG_TOPIC, 10)

        self.get_logger().info(
            f"四轮健康监控: {WHEEL_RPM_TOPIC} + {CMD_VEL_TOPIC} -> {WHEEL_DIAG_TOPIC}"
        )
        self.get_logger().info(
            f"阈值 同侧失配={self.side_mismatch_rpm}rpm 停转={self.stall_rpm}rpm "
            f"确认时长={self.hold_s}s"
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_cmd_vel(self, msg):
        self.cmd_v = float(msg.linear.x)
        self.cmd_w = float(msg.angular.z)
        self.last_cmd_time = self.now_s()

    def cmd_is_fresh(self):
        return (self.last_cmd_time is not None
                and self.now_s() - self.last_cmd_time <= self.cmd_timeout_s)

    def on_wheel_rpm(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warning(f"wheel_rpm 需要 4 个值,收到 {len(msg.data)} 个")
            return

        rpm = [float(value) for value in msg.data[:4]]
        faults = self.evaluate(rpm)
        self.report(self.confirm(faults))

    def evaluate(self, rpm):
        """跑四项判据,返回本帧的瞬时故障码列表(未做持续时间确认)。"""
        faults = []
        commanded = self.cmd_is_fresh()
        should_move = commanded and (abs(self.cmd_v) > self.moving_cmd_mps
                                     or abs(self.cmd_w) > self.straight_cmd_rps)

        # 1. 同侧失配:同侧两轮机械同速,差大 = 有一个在打滑/没着地
        for side_name, front, rear in (("left", FL, RL), ("right", FR, RR)):
            if abs(rpm[front] - rpm[rear]) > self.side_mismatch_rpm:
                faults.append(f"SIDE_MISMATCH:{side_name}")

        # 2. 单轮停转:同侧另一轮明显在转,本轮却不动 → 堵转/断线/编码器故障
        if should_move:
            for wheel, mate in ((FL, RL), (RL, FL), (FR, RR), (RR, FR)):
                if (abs(rpm[wheel]) < self.stall_rpm
                        and abs(rpm[mate]) > self.side_mismatch_rpm):
                    faults.append(f"WHEEL_STALL:{WHEEL_NAMES[wheel]}")

        # 3. 命令为零却在转:被推/溜坡/$STOP 没生效
        if commanded and not should_move:
            if any(abs(value) > self.side_mismatch_rpm for value in rpm):
                faults.append("MOVING_WHILE_IDLE")

        # 4. 直行左右偏:w≈0 时左右侧平均之差,给 w_bias 标定当依据
        if should_move and abs(self.cmd_w) <= self.straight_cmd_rps:
            left_mean = (rpm[FL] + rpm[RL]) * 0.5
            right_mean = (rpm[FR] + rpm[RR]) * 0.5
            delta = left_mean - right_mean
            if abs(delta) > self.side_mismatch_rpm:
                faults.append(f"STRAIGHT_DRIFT:{delta:+.0f}rpm")

        return faults

    def confirm(self, faults):
        """持续时间确认:只有连续出现超过 hold_s 的故障码才算数。"""
        now = self.now_s()
        # STRAIGHT_DRIFT 带数值,按前缀归并,否则数值一变就重新计时、永远确认不了
        keys = {fault.split(":")[0]: fault for fault in faults}

        for key in list(self.fault_since):
            if key not in keys:
                del self.fault_since[key]
        for key in keys:
            self.fault_since.setdefault(key, now)

        return [keys[key] for key in keys
                if now - self.fault_since[key] >= self.hold_s]

    def report(self, confirmed):
        text = ";".join(confirmed) if confirmed else DIAG_OK

        message = String()
        message.data = text
        self.diag_publisher.publish(message)

        if text != self.last_reported:
            if text == DIAG_OK:
                self.get_logger().info("四轮恢复正常")
            else:
                self.get_logger().warning(f"四轮异常: {text}")
            self.last_reported = text


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = WheelHealthMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
