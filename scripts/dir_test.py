#!/usr/bin/env python3
"""dir_test.py —— 在线翻电机方向(motor_dir)并当场低速 $VW 验证,不用重烧固件。

背景:烧 v3 固件时 CAR_PARAM_VERSION 2->3 把 flash 里存的 motor_dir 清回默认(1,1),
导致方向反了。若方向只是被清掉(不是硬件接反),用 $SET,DIR 在线翻回来 + $SAVE 即可。

流程:$MODE,CONFIG -> $SET,DIR,L,R -> [可选 $SAVE] -> $MODE,VW -> 喷 $VW 若干秒 -> $STATUS -> $STOP

用法(**第一次务必把轮子架空**,手放电源开关上):
  python3 dir_test.py                 # 试 motor_dir=(-1,-1),$VW,0.1,0 跑 1s,不保存
  python3 dir_test.py --l -1 --r -1
  python3 dir_test.py --l -1 --r -1 --save   # 方向对且稳定后,加 --save 写进 flash

看运动中 $STATUS + 轮子:
  轮子前进 且 rpm_l/rpm_r 跟随正目标(~+27)稳定  -> 方向修好,加 --save 固化,不用重烧
  轮子还是后退                                    -> 换 --l/--r 符号再试
  rpm 飙到几百 / pwm 冲到 10000(正反馈飞车)      -> 立刻断电,这组 dir 不稳,告诉我(需走 encoder 重烧那条路)
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


def tx(fd, s, wait=0.3):
    os.write(fd, s.encode())
    termios.tcdrain(fd)
    return drain(fd, wait)


def main():
    p = argparse.ArgumentParser(description="在线翻电机方向并低速 $VW 验证")
    p.add_argument("--port", default="/dev/ttyS3")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--l", type=int, default=-1, help="left_motor_dir (1 或 -1)")
    p.add_argument("--r", type=int, default=-1, help="right_motor_dir (1 或 -1)")
    p.add_argument("--v", type=float, default=0.1, help="$VW 线速度 m/s")
    p.add_argument("--secs", type=float, default=1.0, help="喷命令秒数")
    p.add_argument("--save", action="store_true", help="设完方向 $SAVE 写入 flash")
    args = p.parse_args()

    fd = open_uart(args.port, args.baud)
    try:
        print(f"[{args.port} @{args.baud}]  motor_dir=({args.l},{args.r})")
        print("基线:", tx(fd, "$GET,STATUS\r\n") or "<无应答>")
        print("CONFIG:", tx(fd, "$MODE,CONFIG\r\n") or "<无应答>")
        print(f"SET DIR:", tx(fd, f"$SET,DIR,{args.l},{args.r}\r\n") or "<无应答>")
        if args.save:
            print("SAVE:", tx(fd, "$SAVE\r\n") or "<无应答>")
        print("VW 模式:", tx(fd, "$MODE,VW\r\n") or "<无应答>")

        frame = f"$VW,{args.v},0.0\r\n"
        print(f"== 喷 {frame.strip()} 约 {args.secs}s(看轮子方向) ==")
        n = int(args.secs / 0.05)
        for _ in range(n):
            os.write(fd, frame.encode())
            termios.tcdrain(fd)
            drain(fd, 0.02)

        print("运动中:", tx(fd, "$GET,STATUS\r\n") or "<无应答>")
        print("停:", tx(fd, "$STOP\r\n") or "<无应答>")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
