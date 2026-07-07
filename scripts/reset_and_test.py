#!/usr/bin/env python3
"""reset_and_test.py —— 强制把【新编译的默认参数】刷进 flash,再低速 $VW 验证方向/稳定。

用途:烧了新固件但 flash 里存着旧 v3 参数(版本号没变→被判有效→覆盖新默认),
导致改的 encoder_dir/motor_dir 默认值不生效。$RESET_PARAM 把 RAM 重置成新编译默认,
$SAVE 写回 flash 覆盖旧值。之后方向/稳定应符合新代码。

流程:$MODE,CONFIG -> $RESET_PARAM -> $SAVE -> $MODE,VW -> 喷 $VW -> $STATUS -> $STOP

用法(**务必先把轮子架空**,手放电源开关上):
  python3 reset_and_test.py
  python3 reset_and_test.py --v 0.12 --secs 1.5
"""

import argparse
import os
import select
import termios
import time


def open_uart(port, baud):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0
    a[2] |= termios.CLOCAL | termios.CREAD
    a[2] &= ~termios.CSIZE
    a[2] |= termios.CS8
    a[2] &= ~termios.PARENB
    a[2] &= ~termios.CSTOPB
    if hasattr(termios, "CRTSCTS"):
        a[2] &= ~termios.CRTSCTS
    flag = getattr(termios, f"B{baud}")
    a[4] = a[5] = flag
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def drain(fd, t):
    end = time.monotonic() + t
    buf = b""
    while time.monotonic() < end:
        r, _, _ = select.select([fd], [], [], max(0, end - time.monotonic()))
        if r:
            buf += os.read(fd, 1024)
    return buf.decode("ascii", "replace").strip()


def tx(fd, s, wait=0.4):
    os.write(fd, s.encode())
    termios.tcdrain(fd)
    return drain(fd, wait)


def main():
    p = argparse.ArgumentParser(description="强制刷新新默认参数并验证")
    p.add_argument("--port", default="/dev/ttyS3")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--v", type=float, default=0.12)
    p.add_argument("--secs", type=float, default=1.5)
    args = p.parse_args()

    fd = open_uart(args.port, args.baud)
    try:
        print(f"[{args.port} @{args.baud}]")
        print("CONFIG:     ", tx(fd, "$MODE,CONFIG\r\n") or "<无应答>")
        print("RESET_PARAM:", tx(fd, "$RESET_PARAM\r\n") or "<无应答>")
        print("SAVE:       ", tx(fd, "$SAVE\r\n", wait=0.8) or "<无应答>")
        print("VW 模式:    ", tx(fd, "$MODE,VW\r\n") or "<无应答>")

        frame = f"$VW,{args.v},0.0\r\n"
        print(f"== 喷 {frame.strip()} 约 {args.secs}s(看方向) ==")
        n = int(args.secs / 0.05)
        for _ in range(n):
            os.write(fd, frame.encode())
            termios.tcdrain(fd)
            drain(fd, 0.02)

        print("运动中:", tx(fd, "$GET,STATUS\r\n") or "<无应答>")
        print("停:    ", tx(fd, "$STOP\r\n") or "<无应答>")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
