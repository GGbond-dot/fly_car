#!/usr/bin/env python3
"""飞车舵机手动控制工具(SR5E1E3,$SERVO 命令)。

$SERVO 不需要进任何模式,任何状态直接发即可,和 $VW 共用同一条驱动板串口。
协议:  $SERVO,index,angle_deg\r\n   index 1~2, angle 0~180。

用法:
  # 单条:1 号舵机转到 90 度
  python3 servo_test.py 1 90

  # 一次发多条:1 号 90 度、2 号 45 度
  python3 servo_test.py 1 90 2 45

  # 交互模式(逐行输入 "index angle",回车发送,Ctrl-C 退出)
  python3 servo_test.py -i

可选:  --port /dev/ttyS3   --baud 115200
"""

import argparse
import os
import select
import sys
import termios
import time

DEFAULT_PORT = "/dev/ttyS3"   # 飞车 SR5E1E3 驱动板(与底盘 $VW 同口)
DEFAULT_BAUD = 115200
SERVO_MIN_INDEX, SERVO_MAX_INDEX = 1, 2
SERVO_MIN_DEG, SERVO_MAX_DEG = 0, 180
READ_TIMEOUT_S = 0.35


def baud_to_termios(baud):
    name = f"B{baud}"
    if not hasattr(termios, name):
        raise ValueError(f"unsupported baud rate: {baud}")
    return getattr(termios, name)


def configure_uart(fd, baud):
    """原始 8N1,和 chassis_bridge.py 一致。"""
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


def send_servo(fd, index, angle_deg):
    if not (SERVO_MIN_INDEX <= index <= SERVO_MAX_INDEX):
        raise ValueError(f"index 必须在 {SERVO_MIN_INDEX}~{SERVO_MAX_INDEX}: {index}")
    if not (SERVO_MIN_DEG <= angle_deg <= SERVO_MAX_DEG):
        raise ValueError(f"angle 必须在 {SERVO_MIN_DEG}~{SERVO_MAX_DEG}: {angle_deg}")
    frame = f"$SERVO,{int(index)},{int(angle_deg)}\r\n"
    os.write(fd, frame.encode("ascii"))
    termios.tcdrain(fd)
    resp = read_available(fd, READ_TIMEOUT_S)
    text = resp.decode("ascii", errors="replace").rstrip() if resp else "<no response>"
    print(f"TX: {frame.rstrip()!r}   RX: {text!r}")


def parse_pairs(tokens):
    if len(tokens) % 2 != 0:
        raise ValueError("参数必须成对给出: index angle [index angle ...]")
    return [(int(tokens[i]), int(tokens[i + 1])) for i in range(0, len(tokens), 2)]


def main(argv=None):
    parser = argparse.ArgumentParser(description="飞车舵机 $SERVO 控制工具")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("-i", "--interactive", action="store_true",
                        help="交互模式:逐行输入 'index angle' 发送")
    parser.add_argument("pairs", nargs="*", metavar="index angle",
                        help="一对或多对 index angle,如: 1 90 2 45")
    args = parser.parse_args(argv)

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_uart(fd, args.baud)
        print(f"Opened {args.port} at {args.baud} 8N1")

        if args.interactive:
            print("交互模式: 输入 'index angle' 回车发送, Ctrl-C 退出")
            try:
                for line in sys.stdin:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        idx, ang = parse_pairs(line.split())[0]
                        send_servo(fd, idx, ang)
                    except ValueError as exc:
                        print(f"输入无效: {exc}", file=sys.stderr)
            except KeyboardInterrupt:
                print()
            return 0

        if not args.pairs:
            parser.error("请给出 index angle,或用 -i 进交互模式")
        for idx, ang in parse_pairs(args.pairs):
            send_servo(fd, idx, ang)
        return 0
    finally:
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
