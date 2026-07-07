#!/usr/bin/env python3
"""fw_check.py —— 确认 SR5E1E3 板上烧的是不是【带前馈的新固件】。

原理:新固件才有 $SET,FF / $SET,ACCEL 两条命令(旧固件不认识,回 $ERR)。
在 CONFIG 模式下发 ff=0/accel=0(=旧行为,不改任何实际参数),看回复:
  两条都 $OK,SET  -> 新固件已上板 ✔
  任一 $ERR       -> 还是旧固件,没烧成

用法(在【开发板】上):
  python3 fw_check.py                 # 默认 /dev/ttyS3 @115200
  python3 fw_check.py --port /dev/ttyS3
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
    p = argparse.ArgumentParser(description="确认新固件(带前馈)是否已烧上板")
    p.add_argument("--port", default="/dev/ttyS3")
    p.add_argument("--baud", type=int, default=115200)
    args = p.parse_args()

    fd = open_uart(args.port, args.baud)
    try:
        print(f"[{args.port} @{args.baud}]")
        print("进 CONFIG 模式:", tx(fd, "$MODE,CONFIG\r\n") or "<无应答>")

        r_ff = tx(fd, "$SET,FF,L,0,0\r\n") or "<无应答>"
        r_accel = tx(fd, "$SET,ACCEL,0,0\r\n") or "<无应答>"
        print("$SET,FF,L,0,0   ->", r_ff)
        print("$SET,ACCEL,0,0  ->", r_accel)

        # 回到安全模式
        tx(fd, "$MODE,VW\r\n")

        ok = ("OK" in r_ff.upper()) and ("OK" in r_accel.upper())
        print()
        if ok:
            print("✔ 新固件已上板(FF/ACCEL 命令都被识别)。可以进标定了。")
        else:
            print("✘ 还是旧固件 —— FF/ACCEL 未被识别($ERR)。烧录没成,重烧。")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
