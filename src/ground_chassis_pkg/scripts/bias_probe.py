#!/usr/bin/env python3
"""飞车地面底盘 左右轮恒定速度差(转向 bias)开环测量节点。

用途:实测"左轮偏快→车向右偏→直边稳态外扩"。开环命令 $VW(v, w=0) 直线跑一段,
用 carto 的 map←laser_link yaw 变化量算出无指令时的转向漂移率 Ω(rad/s),
取反即为 diff_drive_controller 需要的前馈 w_bias_rps(加到 w 上抵消漂移)。

自动按多个速度各跑一次:
  - 三个速度 Ω 基本相同  -> 恒定 bias,w_bias 前馈全速段可用
  - Ω 随 v 明显增大      -> 轮径/编码器比例误差,需固件/bridge 缩放左轮(前馈只在标定速度准)

链路(本节点只发 /cmd_vel,由 chassis_bridge 转 $VW;需 carto 出 TF):
  fly_carto(TF) + chassis_bridge(/cmd_vel->$VW) + 本节点
配套 launch: my_launch/measure_wheel_bias.launch.py (一条命令起全套)

⚠ 车前方留 2~3m 空地:开环转向漂移会让车走弧线,可能横向甩出 0.5m 量级。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import tf2_ros


def yaw_from_quat(q):
    # z-yaw from quaternion (planar)
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class BiasProbe(Node):
    def __init__(self):
        super().__init__("bias_probe")
        # 逗号分隔的测试速度(m/s);每个速度跑 drive_s 秒
        self.declare_parameter("speeds", "0.10,0.15,0.20")
        self.declare_parameter("drive_s", 5.0)      # 每段开环直线时长
        self.declare_parameter("stop_s", 2.0)       # 每段之间刹停+读数稳定时长
        self.declare_parameter("warmup_s", 8.0)     # 等 carto TF 起来
        self.declare_parameter("rate_hz", 20.0)

        raw = self.get_parameter("speeds").get_parameter_value().string_value
        self.speeds = [float(s) for s in raw.split(",") if s.strip()]
        self.drive_s = self.get_parameter("drive_s").value
        self.stop_s = self.get_parameter("stop_s").value
        self.warmup_s = self.get_parameter("warmup_s").value
        self.dt = 1.0 / max(self.get_parameter("rate_hz").value, 1.0)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.results = []           # (v, omega_measured_rps, dist_m, lat_m)
        self.phase = "warmup"       # warmup -> drive -> stop -> ... -> done
        self.idx = 0
        self.phase_t0 = self.get_clock().now()
        self.start_pose = None      # (x, y, yaw) at drive start
        self.get_logger().info(
            f"bias_probe: speeds={self.speeds} drive={self.drive_s}s "
            f"stop={self.stop_s}s warmup={self.warmup_s}s  (车前方留空地!)")
        self.timer = self.create_timer(self.dt, self.tick)

    def pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "laser_link", Time())
            t = tf.transform.translation
            return (t.x, t.y, yaw_from_quat(tf.transform.rotation))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None

    def send(self, v, w):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.pub.publish(m)

    def elapsed(self):
        return (self.get_clock().now() - self.phase_t0).nanoseconds * 1e-9

    def enter(self, phase):
        self.phase = phase
        self.phase_t0 = self.get_clock().now()

    def tick(self):
        # 全程持续发指令(bridge 看门狗断流会 $STOP)
        if self.phase == "warmup":
            self.send(0.0, 0.0)
            if self.elapsed() >= self.warmup_s:
                if self.pose() is None:
                    self.get_logger().warn("still no TF map<-laser_link, waiting...")
                    self.phase_t0 = self.get_clock().now()  # 再等一轮
                    return
                self.begin_drive()
            return

        if self.phase == "drive":
            v = self.speeds[self.idx]
            self.send(v, 0.0)             # 开环直线:w=0
            if self.elapsed() >= self.drive_s:
                self.enter("stop")
            return

        if self.phase == "stop":
            self.send(0.0, 0.0)
            if self.elapsed() >= self.stop_s:
                self.finish_run()
            return

        if self.phase == "done":
            self.send(0.0, 0.0)
            return

    def begin_drive(self):
        p = self.pose()
        self.start_pose = p
        self.get_logger().info(
            f"--- run {self.idx + 1}/{len(self.speeds)}  v={self.speeds[self.idx]:.2f} m/s "
            f"start yaw={math.degrees(p[2]):+.1f}° ---")
        self.enter("drive")

    def finish_run(self):
        p = self.pose()
        v = self.speeds[self.idx]
        if p is not None and self.start_pose is not None:
            x0, y0, yaw0 = self.start_pose
            x1, y1, yaw1 = p
            dyaw = norm_angle(yaw1 - yaw0)
            omega = dyaw / self.drive_s            # 实测开环转向漂移率 rad/s
            dist = math.hypot(x1 - x0, y1 - y0)
            # 横向漂移量(相对起点朝向的垂直分量)
            dx, dy = x1 - x0, y1 - y0
            lat = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
            self.results.append((v, omega, dist, lat))
            self.get_logger().info(
                f"    Δyaw={math.degrees(dyaw):+.1f}°  Ω={omega:+.4f} rad/s "
                f"({math.degrees(omega):+.1f}°/s)  行程={dist:.2f}m  横漂={lat:+.2f}m")
        else:
            self.get_logger().warn(f"    v={v:.2f}: 无 TF,跳过")

        self.idx += 1
        if self.idx < len(self.speeds):
            self.begin_drive()
        else:
            self.report()
            self.enter("done")

    def report(self):
        self.get_logger().info("========== 测量汇总 ==========")
        if not self.results:
            self.get_logger().warn("无有效数据(TF 没起来?)")
            return
        omegas = [r[1] for r in self.results]
        for v, om, dist, lat in self.results:
            r_turn = (v / abs(om)) if abs(om) > 1e-6 else float("inf")
            self.get_logger().info(
                f"  v={v:.2f}  Ω={om:+.4f} rad/s  转弯半径≈{r_turn:.2f}m  横漂={lat:+.2f}m")
        mean_om = sum(omegas) / len(omegas)
        spread = max(omegas) - min(omegas)
        w_bias = -mean_om        # 前馈取反抵消漂移
        self.get_logger().info(
            f"  平均 Ω={mean_om:+.4f} rad/s → 建议 w_bias_rps={w_bias:+.4f}")
        # 恒定 vs 比例判据:各速 Ω 离散度相对均值
        rel = spread / abs(mean_om) if abs(mean_om) > 1e-6 else 0.0
        if rel < 0.25:
            self.get_logger().info(
                f"  各速 Ω 一致(离散 {rel*100:.0f}%)→ 恒定 bias,w_bias 前馈全速段可用。")
        else:
            self.get_logger().info(
                f"  各速 Ω 差异大(离散 {rel*100:.0f}%)→ 疑似比例误差(轮径/编码器),"
                f"前馈只在标定速度准,根治需缩放左轮。")
        self.get_logger().info(
            f"  用法: ros2 param set /diff_drive_controller w_bias_rps {w_bias:.4f}")
        self.get_logger().info("=============================")


def main():
    rclpy.init()
    node = BiasProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前兜底刹停
        try:
            node.send(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
