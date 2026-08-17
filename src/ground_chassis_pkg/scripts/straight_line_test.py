#!/usr/bin/env python3
"""四驱底盘"走不走得直"单脚本测试 —— 自己跑、自己判、自己出结论表。

背景:换四驱的唯一目的就是"不要走得很歪"。这个脚本把那件事量化成一次
可复现的测量,跑完直接打 ✅/❌ 表,不用回头翻日志、贴给别人看。

测的是**开环直线**:自己按固定速度发 cmd_vel,绕开 diff_drive_controller。
为什么不带闭环测:闭环会把机械的歪自动纠回来,测出来的是"控制器行不行",
而现在要回答的是"四驱底盘本身直不直"。底盘直了闭环才有意义;底盘歪着靠
闭环硬拉,直线上会看到持续的修正抖动(就是之前那个画龙)。

  测量基准 TF map -> laser_link(carto),与 diff_drive_controller 同源。

五项指标:
  横向偏移    终点偏离"起点位姿指向的那条直线"多少 cm —— 这就是"走歪"本身
  航向漂移率  yaw 变化 / 时长 rad/s —— 换算成弧线半径,给 w_bias_rps 定初值
  行进距离    实测 vs 命令,差 >5% 说明轮径/ppr 标定不对(尺子歪了,别的都白测)
  同侧失配    max|fl-rl|、max|fr-rr| —— 非零说明有轮打滑/虚接触(四驱特有)
  左右侧差    左右平均 rpm 之差 —— 恒定速度差,直接换算 w_bias_rps

用法(飞车板,先起 chassis_bridge):
  ros2 run ground_chassis_pkg straight_line_test.py --ros-args -p distance_cm:=200.0
或用 launch 一条命令起全套:
  ros2 launch ground_chassis_pkg test_straight_line.launch.py

⚠ 车前方留出 距离+1m 空地。开环走歪时横向可能甩出几十 cm。
"""

import math
import sys

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32MultiArray
    from tf2_ros import Buffer, TransformListener
    import tf2_ros
except ImportError:
    print(
        "ERROR: 需要在 ROS2 环境中运行,请先 source /opt/ros/<distro>/setup.bash",
        file=sys.stderr,
    )
    raise


CMD_VEL_TOPIC = "cmd_vel"
WHEEL_RPM_TOPIC = "/chassis/wheel_rpm"
MAP_FRAME = "map"
BASE_FRAME = "laser_link"

FL, FR, RL, RR = 0, 1, 2, 3
PASS_MARK, FAIL_MARK, SKIP_MARK = "✅", "❌", "—"


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class StraightLineTest(Node):
    def __init__(self):
        super().__init__("straight_line_test")

        self.declare_parameter("distance_cm", 200.0)
        self.declare_parameter("speed_mps", 0.16)
        self.declare_parameter("rate_hz", 20.0)
        # 判据阈值(可按现场要求松紧)
        self.declare_parameter("max_lateral_cm", 10.0)      # 2m 内偏 >10cm 判不合格
        self.declare_parameter("max_yaw_drift_rps", 0.03)   # 半径 ~5m 以内算歪
        self.declare_parameter("max_distance_error_pct", 5.0)
        self.declare_parameter("max_side_mismatch_rpm", 24.0)
        self.declare_parameter("settle_s", 1.0)             # 起步/停车前后静置

        self.distance_m = self.get_parameter("distance_cm").value / 100.0
        self.speed = self.get_parameter("speed_mps").value
        self.rate = self.get_parameter("rate_hz").value
        self.max_lateral_cm = self.get_parameter("max_lateral_cm").value
        self.max_yaw_drift = self.get_parameter("max_yaw_drift_rps").value
        self.max_distance_error_pct = self.get_parameter("max_distance_error_pct").value
        self.max_side_mismatch = self.get_parameter("max_side_mismatch_rpm").value
        self.settle_s = self.get_parameter("settle_s").value

        self.duration = self.distance_m / self.speed if self.speed > 0 else 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(
            Float32MultiArray, WHEEL_RPM_TOPIC, self.on_wheel_rpm, 10)

        self.start_pose = None
        self.end_pose = None
        self.start_time = None
        self.phase = "settle_start"
        self.phase_time = None
        self.finished = False

        # 四轮统计(只在行进段累计)
        self.collecting = False
        self.max_left_mismatch = 0.0
        self.max_right_mismatch = 0.0
        self.left_sum = 0.0
        self.right_sum = 0.0
        self.rpm_samples = 0

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f"开环直线测试: v={self.speed}m/s 距离={self.distance_m*100:.0f}cm "
            f"预计{self.duration:.1f}s"
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_wheel_rpm(self, msg):
        if not self.collecting or len(msg.data) < 4:
            return
        rpm = [float(value) for value in msg.data[:4]]
        self.max_left_mismatch = max(self.max_left_mismatch, abs(rpm[FL] - rpm[RL]))
        self.max_right_mismatch = max(self.max_right_mismatch, abs(rpm[FR] - rpm[RR]))
        self.left_sum += (rpm[FL] + rpm[RL]) * 0.5
        self.right_sum += (rpm[FR] + rpm[RR]) * 0.5
        self.rpm_samples += 1

    def lookup_pose(self):
        """取 map->laser_link 的 (x, y, yaw);拿不到返回 None。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, BASE_FRAME, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = tf.transform.translation
        return (t.x, t.y, yaw_from_quaternion(tf.transform.rotation))

    def publish(self, v):
        message = Twist()
        message.linear.x = v
        self.publisher.publish(message)

    def tick(self):
        if self.finished:
            return
        now = self.now_s()
        if self.phase_time is None:
            self.phase_time = now

        if self.phase == "settle_start":
            # 静置取起点位姿,避免起步瞬间的 carto 抖动进基准
            self.publish(0.0)
            if now - self.phase_time < self.settle_s:
                return
            self.start_pose = self.lookup_pose()
            if self.start_pose is None:
                self.get_logger().warning(
                    f"等待 TF {MAP_FRAME}->{BASE_FRAME}(carto 起了吗?)")
                self.phase_time = now
                return
            self.phase, self.phase_time, self.start_time = "drive", now, now
            self.collecting = True
            self.get_logger().info("开始行进")

        elif self.phase == "drive":
            if now - self.start_time < self.duration:
                self.publish(self.speed)
                return
            self.publish(0.0)
            self.collecting = False
            self.phase, self.phase_time = "settle_end", now
            self.get_logger().info("到达,静置取终点位姿")

        elif self.phase == "settle_end":
            self.publish(0.0)
            if now - self.phase_time < self.settle_s:
                return
            self.end_pose = self.lookup_pose()
            self.finished = True
            self.print_verdict()

    def print_verdict(self):
        if self.start_pose is None or self.end_pose is None:
            self.get_logger().error("拿不到起点/终点位姿,无法出结论")
            return

        x0, y0, yaw0 = self.start_pose
        x1, y1, yaw1 = self.end_pose
        dx, dy = x1 - x0, y1 - y0

        # 以起点航向为基准分解:沿轴=前进距离,垂轴=横向偏移(就是"走歪")
        forward_m = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        lateral_m = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        yaw_drift = normalize_angle(yaw1 - yaw0)
        yaw_rate = yaw_drift / self.duration if self.duration > 0 else 0.0
        distance_error_pct = (
            (forward_m - self.distance_m) / self.distance_m * 100.0
            if self.distance_m > 0 else 0.0)

        rows = []
        rows.append(self.row(
            "横向偏移", f"{lateral_m*100:+.1f} cm",
            abs(lateral_m * 100) <= self.max_lateral_cm,
            f"≤{self.max_lateral_cm:.0f}cm"))
        rows.append(self.row(
            "航向漂移率", f"{yaw_rate:+.4f} rad/s",
            abs(yaw_rate) <= self.max_yaw_drift,
            f"≤{self.max_yaw_drift:.3f}"))
        rows.append(self.row(
            "行进距离", f"{forward_m*100:.1f} cm ({distance_error_pct:+.1f}%)",
            abs(distance_error_pct) <= self.max_distance_error_pct,
            f"±{self.max_distance_error_pct:.0f}%"))

        if self.rpm_samples > 0:
            worst = max(self.max_left_mismatch, self.max_right_mismatch)
            rows.append(self.row(
                "同侧失配", f"左{self.max_left_mismatch:.0f} 右{self.max_right_mismatch:.0f} rpm",
                worst <= self.max_side_mismatch,
                f"≤{self.max_side_mismatch:.0f}rpm"))
            side_delta = (self.left_sum - self.right_sum) / self.rpm_samples
            rows.append(self.row(
                "左右侧差", f"{side_delta:+.1f} rpm(左-右)",
                abs(side_delta) <= self.max_side_mismatch,
                f"≤{self.max_side_mismatch:.0f}rpm"))
        else:
            rows.append((SKIP_MARK, "四轮 rpm", "无数据($RPM4 没上报)", "—"))

        lines = ["", "=" * 62, "  四驱开环直线测试结论", "=" * 62]
        lines.append(f"  {'项目':<12}{'实测':<28}{'判据':<12}")
        lines.append("-" * 62)
        for mark, name, value, criterion in rows:
            lines.append(f"{mark} {name:<12}{value:<28}{criterion:<12}")
        lines.append("-" * 62)

        hard_failed = [name for mark, name, _, _ in rows if mark == FAIL_MARK]
        if hard_failed:
            lines.append(f"{FAIL_MARK} 不合格: {', '.join(hard_failed)}")
            lines.extend(self.advise(rows, yaw_rate))
        else:
            lines.append(f"{PASS_MARK} 全部通过 —— 底盘开环走直合格,可以接闭环")
        lines.append("=" * 62)
        lines.append("")
        print("\n".join(lines))

    @staticmethod
    def row(name, value, ok, criterion):
        return (PASS_MARK if ok else FAIL_MARK, name, value, criterion)

    def advise(self, rows, yaw_rate):
        """把不合格项翻译成下一步动作,省得跑完还要人去想怎么办。"""
        failed = {name for mark, name, _, _ in rows if mark == FAIL_MARK}
        tips = ["", "下一步:"]
        if "同侧失配" in failed:
            tips.append("  · 同侧两轮转速对不上 = 有轮打滑或没着地。先查机械四轮共面,")
            tips.append("    别急着调控制参数 —— 这一项不清零,下面几项测出来都不作数。")
        if "行进距离" in failed:
            tips.append("  · 距离对不上 = 轮径/ppr 标定不对。$SET,WHEEL 改轮径把残差折进去,")
            tips.append("    尺子没校准前,横向偏移和漂移率都不可信。")
        if "航向漂移率" in failed or "横向偏移" in failed:
            radius = self.speed / abs(yaw_rate) if abs(yaw_rate) > 1e-6 else float("inf")
            tips.append(f"  · 走弧线,半径约 {radius:.1f}m。若同侧失配和距离都合格,说明是")
            tips.append("    左右侧恒定速度差 —— 把 diff_drive_controller 的 w_bias_rps")
            tips.append(f"    设成 {-yaw_rate:+.4f} 补掉,再跑一次本测试确认。")
        if "左右侧差" in failed:
            tips.append("  · 左右侧平均 rpm 差得多 = 两侧电机/机械不对称,优先从机械和")
            tips.append("    $SET,FF 左右前馈上找,靠 w_bias 硬补会牺牲平顺。")
        return tips


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = StraightLineTest()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish(0.0)   # 退出前务必刹停
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
