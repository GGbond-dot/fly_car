#!/usr/bin/env python3
"""飞车地面差速底盘串口桥(SR5E1E3,$VW 流式通道)。

仿照 car/orangepi_to_car/car/orangepi_to_carv2.py 的 cmd_vel→$VW 快速通道与看门狗,
但只保留地面跟踪需要的部分:订阅 geometry_msgs/Twist 的 cmd_vel,
  linear.x = v (m/s), angular.z = w (rad/s) → 进入 VW 模式后持续发 $VW,v,w
去掉了 car 版里跟随任务无关的舵机扫动与 car_movement 离散命令。

  - 流式发送不逐帧等应答(快速通道),发送间隔下限 VW_MIN_INTERVAL_S;
  - cmd_vel 断流超过 CMD_VEL_TIMEOUT_S 自动发 $STOP(桥侧看门狗);
  - --chassis-timeout-ms > 0 时启动后下发 $SET,TIMEOUT,1,ms 启用底盘侧通信超时兜底。

只发送 SR5E1E3 协议中已写明的核心串口帧($MODE,VW / $VW / $STOP / $SET)。
"""

import argparse
import os
import select
import sys
import termios
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
except ImportError:
    print(
        "ERROR: 需要在 ROS2 环境中运行,请先 source /opt/ros/<distro>/setup.bash",
        file=sys.stderr,
    )
    raise


DEFAULT_PORT = "/dev/ttyS6"   # 与 car 同款 SR5E1E3 板
DEFAULT_BAUD = 115200
CMD_VEL_TOPIC = "cmd_vel"

# v 限幅 m/s,w 限幅 rad/s(与 car 版一致)
V_MIN, V_MAX = -2.0, 2.0
W_MAX = 3.0

VW_MIN_INTERVAL_S = 0.04     # $VW 流式发送的最小间隔(约 25Hz 上限)
VW_DRAIN_TIMEOUT_S = 0.02    # 快速通道只顺手清空接收缓冲,不等待应答
CMD_VEL_TIMEOUT_S = 0.5      # cmd_vel 断流判定,超时发 $STOP
CMD_VEL_WATCHDOG_PERIOD_S = 0.1
READ_TIMEOUT_S = 0.35

CMD_STOP = "$STOP\r\n"
CMD_VW_MODE = "$MODE,VW\r\n"


def baud_to_termios(baud):
    constant_name = f"B{baud}"
    if not hasattr(termios, constant_name):
        raise ValueError(f"unsupported baud rate: {baud}")
    return getattr(termios, constant_name)


def configure_uart(fd, baud):
    """配置串口为 SR5E1E3 要求的原始 8N1 模式。"""
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[3] = 0
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSTOPB
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    baud_flag = baud_to_termios(baud)
    attrs[4] = baud_flag
    attrs[5] = baud_flag
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def read_available(fd, timeout_s):
    """读取 timeout_s 内已到达的所有串口字节(顺手清缓冲用)。"""
    deadline = time.monotonic() + timeout_s
    chunks = []
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            break
        try:
            data = os.read(fd, 1024)
        except BlockingIOError:
            continue
        if not data:
            break
        chunks.append(data)
    return b"".join(chunks)


def format_float(value):
    return f"{float(value):.3f}".rstrip("0").rstrip(".")


def make_vw_frame(v, w):
    return f"$VW,{format_float(v)},{format_float(w)}\r\n"


class ChassisBridgeNode(Node):
    def __init__(self, port, baud, chassis_timeout_ms=0):
        super().__init__("chassis_bridge")
        self.fd = None
        self.vw_stream_active = False
        self.last_cmd_vel_time = None
        self.last_vw_send_time = 0.0
        self.last_vw_nonzero = False

        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            configure_uart(self.fd, baud)
        except Exception:
            os.close(self.fd)
            self.fd = None
            raise

        if chassis_timeout_ms > 0:
            # 底盘侧通信超时兜底:静默 chassis_timeout_ms 后底盘自动刹停
            self.write_frame(f"$SET,TIMEOUT,1,{int(chassis_timeout_ms)}\r\n")

        self.cmd_vel_subscription = self.create_subscription(
            Twist, CMD_VEL_TOPIC, self.on_cmd_vel, 10,
        )
        self.cmd_vel_watchdog = self.create_timer(
            CMD_VEL_WATCHDOG_PERIOD_S, self.check_cmd_vel_timeout,
        )
        self.get_logger().info(f"Opened {port} at {baud} 8N1")
        self.get_logger().info(
            "Topic cmd_vel Twist: linear.x=v m/s, angular.z=w rad/s -> $VW stream"
        )

    def write_frame(self, frame):
        os.write(self.fd, frame.encode("ascii"))
        termios.tcdrain(self.fd)
        read_available(self.fd, VW_DRAIN_TIMEOUT_S)

    def on_cmd_vel(self, msg):
        v = max(V_MIN, min(V_MAX, float(msg.linear.x)))
        w = max(-W_MAX, min(W_MAX, float(msg.angular.z)))
        is_zero = v == 0.0 and w == 0.0

        now = time.monotonic()
        self.last_cmd_vel_time = now

        # 限频:非零帧间隔不足时丢弃;零速帧在上一帧非零时必须放行(保证刹停到位)
        if now - self.last_vw_send_time < VW_MIN_INTERVAL_S:
            if not (is_zero and self.last_vw_nonzero):
                return

        if not self.vw_stream_active:
            self.write_frame(CMD_VW_MODE)
            self.vw_stream_active = True

        self.write_frame(make_vw_frame(v, w))
        self.last_vw_send_time = now
        self.last_vw_nonzero = not is_zero

    def check_cmd_vel_timeout(self):
        """桥侧看门狗:cmd_vel 断流即刹停。"""
        if not self.vw_stream_active or self.last_cmd_vel_time is None:
            return
        if time.monotonic() - self.last_cmd_vel_time > CMD_VEL_TIMEOUT_S:
            self.get_logger().warning("cmd_vel timeout -> $STOP")
            self.write_frame(CMD_STOP)
            self.vw_stream_active = False
            self.last_cmd_vel_time = None
            self.last_vw_nonzero = False

    def close(self):
        if self.fd is not None:
            try:
                self.write_frame(CMD_STOP)
            except OSError as exc:
                self.get_logger().error(f"Failed to send $STOP on close: {exc}")
            os.close(self.fd)
            self.fd = None


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="飞车地面差速底盘 $VW 串口桥。")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"UART 设备,默认 {DEFAULT_PORT}")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"波特率,默认 {DEFAULT_BAUD}")
    parser.add_argument(
        "--chassis-timeout-ms", type=int, default=0,
        help="启用底盘侧通信超时 ($SET,TIMEOUT,1,ms);0 关闭(默认)",
    )
    return parser.parse_known_args(argv)


def main(argv=None):
    parsed_args, ros_args = parse_args(argv)
    rclpy.init(args=ros_args if ros_args else None)
    node = None
    exit_code = 0
    try:
        node = ChassisBridgeNode(
            parsed_args.port, parsed_args.baud, parsed_args.chassis_timeout_ms,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"chassis_bridge fatal: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
