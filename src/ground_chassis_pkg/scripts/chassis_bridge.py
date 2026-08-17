#!/usr/bin/env python3
"""飞车地面差速底盘串口桥(SR5E1E3,$VW 流式通道 + $RPM4 四轮上行)。

仿照 car/orangepi_to_car/car/orangepi_to_carv2.py 的 cmd_vel→$VW 快速通道与看门狗,
但只保留地面跟踪需要的部分:订阅 geometry_msgs/Twist 的 cmd_vel,
  linear.x = v (m/s), angular.z = w (rad/s) → 进入 VW 模式后持续发 $VW,v,w
去掉了 car 版里跟随任务无关的舵机扫动与 car_movement 离散命令。

  - 流式发送不逐帧等应答(快速通道),发送间隔下限 VW_MIN_INTERVAL_S;
  - cmd_vel 断流超过 CMD_VEL_TIMEOUT_S 自动发 $STOP(桥侧看门狗);
  - --chassis-timeout-ms > 0 时启动后下发 $SET,TIMEOUT,1,ms 启用底盘侧通信超时兜底。

四驱改造(下行协议不变,只加上行):
  四轮差速对上层仍是差速,v/w 语义不变 → $VW 下行零改动,四轮解算留在固件里。
  底盘周期主动上报 `$RPM4,fl,fr,rl,rr`(转/分),桥解析后发 /chassis/wheel_rpm。
  用主动上报而不是 $GET 轮询:轮询要占串口往返,会跟 20Hz 的 $VW 流抢带宽。
  ⚠ 编码器实测最高 50Hz(20ms),ppr=255 → 巡航 66rpm 时每拍仅 ~5.6 计数,
    量化噪声 ±18%。所以 wheel_rpm **不适合做控制内环的权威反馈**,定位是
    诊断量(喂 wheel_health_monitor 判某轮打滑/虚接触);走直线的权威仍是
    ROS 侧 diff_drive_controller 的 carto 航向闭环。

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
    from std_msgs.msg import Float32MultiArray, Int16MultiArray
except ImportError:
    print(
        "ERROR: 需要在 ROS2 环境中运行,请先 source /opt/ros/<distro>/setup.bash",
        file=sys.stderr,
    )
    raise


DEFAULT_PORT = "/dev/ttyS3"   # 飞车地面底盘 SR5E1E3 板(与 car 同款,但飞车接在 ttyS3)
DEFAULT_BAUD = 115200
CMD_VEL_TOPIC = "cmd_vel"
SERVO_CMD_TOPIC = "servo_cmd"   # Int16MultiArray [index, angle_deg] -> $SERVO(与 $VW 共用本串口)
WHEEL_RPM_TOPIC = "/chassis/wheel_rpm"   # Float32MultiArray [fl, fr, rl, rr] 转/分

# 四轮上报:$RPM4,fl,fr,rl,rr。轮序固定 左前/右前/左后/右后,与固件一致。
RPM4_PREFIX = "$RPM4,"
WHEEL_COUNT = 4
WHEEL_NAMES = ("fl", "fr", "rl", "rr")
# 底盘上报周期。编码器实测上限 50Hz → 20ms 是能取到的最快值,再快固件也刷不出新数。
DEFAULT_RPM4_PERIOD_MS = 20
SERIAL_POLL_PERIOD_S = 0.01   # 串口收包轮询(100Hz,快于 50Hz 上报,不漏帧)
RX_BUFFER_MAX_BYTES = 4096    # 行缓冲上限,防上位机卡顿时无限增长
RPM4_STALE_S = 1.0            # 超过此时长没收到 $RPM4 判为上报中断

# 舵机限位($SERVO,index 1~2,angle 0~180;与 fly_car/scripts/servo_test.py 一致)
SERVO_MIN_INDEX, SERVO_MAX_INDEX = 1, 2
SERVO_MIN_DEG, SERVO_MAX_DEG = 0, 180

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


def decode_response(response):
    return response.decode("ascii", errors="replace").rstrip()


def classify_response(response):
    """按 SR5E1E3 协议粗分驱动板回复:无回复/报错/确认/仅有数据。"""
    if not response:
        return "NO_RESPONSE"
    text = decode_response(response)
    if "$ERR," in text:
        return "ERR"
    if "$OK," in text:
        return "OK"
    return "RX_ONLY"


class CommandResult:
    """一条串口命令的返回结果,供进 VW 模式等握手判断是否成功。"""

    def __init__(self, command, response):
        self.command = command
        self.response = response
        self.text = decode_response(response) if response else ""
        self.status = classify_response(response)

    @property
    def can_continue(self):
        # 只有明确 NO_RESPONSE / ERR 才算失败;RX_ONLY / OK 都放行
        return self.status not in ("NO_RESPONSE", "ERR")


def send_command(fd, command, read_timeout_s=READ_TIMEOUT_S, logger=None):
    """发一帧已带 CRLF 的命令并读驱动板回复(慢通道,握手用)。"""
    if logger is not None:
        logger.info(f"TX: {command.rstrip()!r}")
    os.write(fd, command.encode("ascii"))
    termios.tcdrain(fd)
    response = read_available(fd, read_timeout_s)
    result = CommandResult(command, response)
    if logger is not None:
        if response:
            logger.info(f"RX: {decode_response(response)!r} [{result.status}]")
        else:
            logger.info(f"RX: <no response> [{result.status}]")
    return result


class ChassisBridgeNode(Node):
    def __init__(self, port, baud, chassis_timeout_ms=0,
                 rpm4_period_ms=DEFAULT_RPM4_PERIOD_MS):
        super().__init__("chassis_bridge")
        self.fd = None
        self.vw_stream_active = False
        self.last_cmd_vel_time = None
        self.last_vw_send_time = 0.0
        self.last_vw_nonzero = False
        self.rx_buffer = b""
        self.last_rpm4_time = None
        self.rpm4_warned_stale = False

        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            configure_uart(self.fd, baud)
        except Exception:
            os.close(self.fd)
            self.fd = None
            raise

        if chassis_timeout_ms > 0:
            # 底盘侧通信超时兜底:静默 chassis_timeout_ms 后底盘自动刹停
            self.send_command(f"$SET,TIMEOUT,1,{int(chassis_timeout_ms)}\r\n")

        self.wheel_rpm_publisher = self.create_publisher(
            Float32MultiArray, WHEEL_RPM_TOPIC, 10,
        )
        if rpm4_period_ms > 0:
            # 开底盘周期上报。固件未实现该命令时会回 $ERR,不致命 —— 桥照常跑,
            # 只是 /chassis/wheel_rpm 没数据,wheel_health_monitor 会报"上报中断"。
            result = self.send_command(f"$SET,RPT,RPM4,{int(rpm4_period_ms)}\r\n")
            if result.status == "ERR":
                self.get_logger().warning(
                    "$SET,RPT,RPM4 被底盘拒绝(固件可能还没实现四轮上报),"
                    "四轮诊断不可用,$VW 控制不受影响"
                )
        # 串口收包轮询:$VW 快通道只顺手清缓冲,单靠它收不全 50Hz 的上报
        self.serial_poll_timer = self.create_timer(
            SERIAL_POLL_PERIOD_S, self.poll_serial,
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist, CMD_VEL_TOPIC, self.on_cmd_vel, 10,
        )
        # 投货舵机:任务节点发 [index, angle] → $SERVO。$SERVO 任何状态可发,不进 VW 模式,
        # 与 $VW 共用本串口;单线程 executor 回调串行,与 $VW 写不并发。
        self.servo_subscription = self.create_subscription(
            Int16MultiArray, SERVO_CMD_TOPIC, self.on_servo_cmd, 10,
        )
        self.cmd_vel_watchdog = self.create_timer(
            CMD_VEL_WATCHDOG_PERIOD_S, self.check_cmd_vel_timeout,
        )
        self.get_logger().info(f"Opened {port} at {baud} 8N1")
        self.get_logger().info(
            "Topic cmd_vel Twist: linear.x=v m/s, angular.z=w rad/s -> $VW stream"
        )
        if rpm4_period_ms > 0:
            self.get_logger().info(
                f"$RPM4 上报 {rpm4_period_ms}ms -> {WHEEL_RPM_TOPIC} [fl,fr,rl,rr]"
            )

    def send_command(self, command, read_timeout_s=READ_TIMEOUT_S):
        """慢通道:发一帧并读回驱动板应答(带 TX/RX 日志),握手/刹停用。

        应答里会混进周期上报的 $RPM4,所以读到的字节同样喂给行解析器,
        否则握手期间的上报会被吞掉(classify_response 只看 $OK,/$ERR,,不受影响)。
        """
        result = send_command(
            self.fd, command, read_timeout_s=read_timeout_s, logger=self.get_logger(),
        )
        self.feed_rx(result.response)
        return result

    def send_command_fast(self, frame):
        """快通道:写出后只顺手清空接收缓冲,不等应答(20Hz $VW 流用)。"""
        os.write(self.fd, frame.encode("ascii"))
        termios.tcdrain(self.fd)
        self.feed_rx(read_available(self.fd, VW_DRAIN_TIMEOUT_S))

    def poll_serial(self):
        """定时收包:把已到达的字节喂进行解析器,并检查上报是否中断。"""
        self.feed_rx(read_available(self.fd, 0.0))

        if self.last_rpm4_time is None:
            return
        if time.monotonic() - self.last_rpm4_time > RPM4_STALE_S:
            if not self.rpm4_warned_stale:
                self.get_logger().warning(
                    f"$RPM4 上报中断 >{RPM4_STALE_S}s,四轮诊断失效($VW 控制不受影响)"
                )
                self.rpm4_warned_stale = True
        else:
            self.rpm4_warned_stale = False

    def feed_rx(self, data):
        """把串口字节按行切分后交给 handle_line;非完整行留在缓冲里等下次。"""
        if not data:
            return
        self.rx_buffer += data
        if len(self.rx_buffer) > RX_BUFFER_MAX_BYTES:
            # 只可能发生在长时间不解析(如被阻塞)时,丢旧留新,保证还能对齐到下一行
            self.rx_buffer = self.rx_buffer[-RX_BUFFER_MAX_BYTES:]

        while True:
            index = self.rx_buffer.find(b"\n")
            if index < 0:
                break
            line = self.rx_buffer[:index]
            self.rx_buffer = self.rx_buffer[index + 1:]
            self.handle_line(line.decode("ascii", errors="replace").strip())

    def handle_line(self, line):
        """解析一行上行帧。目前只关心 $RPM4,其余(含 $OK/$ERR)交给慢通道判定。"""
        if not line.startswith(RPM4_PREFIX):
            return

        fields = line[len(RPM4_PREFIX):].split(",")
        if len(fields) < WHEEL_COUNT:
            self.get_logger().warning(f"$RPM4 字段不足 4 个,丢弃: {line!r}")
            return
        try:
            rpm = [float(field) for field in fields[:WHEEL_COUNT]]
        except ValueError:
            self.get_logger().warning(f"$RPM4 数值解析失败,丢弃: {line!r}")
            return

        message = Float32MultiArray()
        message.data = rpm
        self.wheel_rpm_publisher.publish(message)
        self.last_rpm4_time = time.monotonic()

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

        # 进 VW 模式必须验证驱动板接受(读 $OK),否则后续 $VW 会被在非 VW 模式下丢弃、
        # 车永远不动。没接受就不置位 vw_stream_active,下一帧 cmd_vel 会自动重试进模式。
        if not self.vw_stream_active:
            result = self.send_command(CMD_VW_MODE)
            if not result.can_continue:
                self.get_logger().warning(
                    "cmd_vel: VW 模式未被驱动板接受(无应答/报错),丢弃本帧,下次重试"
                )
                return
            self.vw_stream_active = True

        self.send_command_fast(make_vw_frame(v, w))
        self.last_vw_send_time = now
        self.last_vw_nonzero = not is_zero

    def on_servo_cmd(self, msg):
        """投货舵机命令 [index, angle_deg] → $SERVO,index,angle。快通道写,不等应答。"""
        if len(msg.data) < 2:
            self.get_logger().warning(f"servo_cmd 需要 [index, angle],收到 {list(msg.data)}")
            return
        index = int(msg.data[0])
        angle = int(msg.data[1])
        if not (SERVO_MIN_INDEX <= index <= SERVO_MAX_INDEX):
            self.get_logger().warning(f"servo_cmd index={index} 越界 [{SERVO_MIN_INDEX},{SERVO_MAX_INDEX}]")
            return
        angle = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, angle))
        self.send_command_fast(f"$SERVO,{index},{angle}\r\n")
        self.get_logger().info(f"servo_cmd -> $SERVO,{index},{angle}")

    def check_cmd_vel_timeout(self):
        """桥侧看门狗:cmd_vel 断流即刹停。"""
        if not self.vw_stream_active or self.last_cmd_vel_time is None:
            return
        if time.monotonic() - self.last_cmd_vel_time > CMD_VEL_TIMEOUT_S:
            self.get_logger().warning("cmd_vel timeout -> $STOP")
            self.send_command(CMD_STOP)
            self.vw_stream_active = False
            self.last_cmd_vel_time = None
            self.last_vw_nonzero = False

    def close(self):
        if self.fd is not None:
            try:
                self.send_command(CMD_STOP)
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
    parser.add_argument(
        "--rpm4-period-ms", type=int, default=DEFAULT_RPM4_PERIOD_MS,
        help=f"四轮 rpm 周期上报 ($SET,RPT,RPM4,ms);0 关闭。默认 {DEFAULT_RPM4_PERIOD_MS}"
             "(编码器实测上限 50Hz,再快也刷不出新数)",
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
            parsed_args.rpm4_period_ms,
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
