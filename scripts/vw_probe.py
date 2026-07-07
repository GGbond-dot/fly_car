#!/usr/bin/env python3
"""飞车地面底盘 $VW 电机诊断脚本(不依赖 ROS,直接串口)。

用途:chassis_bridge 已能和 SR5E1E3 板正常通信($OK,MODE),但车不动时,
用它绕开整个 ROS 链,直接对板子发 $VW 并用 $GET,STATUS 读回真实状态,
一刀切定位卡在哪一环(命令没解析 / 驱动没使能 / 电机没转 / 电源不足 / 故障)。

用法:
  python3 vw_probe.py                 # 默认 /dev/ttyS3 @115200,喷 $VW,0.15,0 两秒
  python3 vw_probe.py --port /dev/ttyS3 --v 0.2 --secs 3
  python3 vw_probe.py --pwm            # 改用 $MODE,PWM + $PWM 开环测试(绕开 PID/编码器)

看运动中那行 $STATUS:
  tgt_l/tgt_r=0        -> 板子没把命令解析成目标速度(格式/模式问题)
  tgt 非零 pwm=0       -> 驱动没使能(看 mode 是不是 DISABLE)
  pwm 非零 rpm=0       -> 电机没转:电源不足/电机没接好/编码器没读数/轮子卡住
  bat 很低             -> 电机动力电源没上或电量不足
  fault != 0x00000000  -> 硬件故障,按故障码查
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
    p = argparse.ArgumentParser(description="飞车底盘 $VW/$PWM 电机诊断")
    p.add_argument("--port", default="/dev/ttyS3")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--v", type=float, default=0.15, help="$VW 线速度 m/s(或 $PWM 占空比)")
    p.add_argument("--w", type=float, default=0.0, help="$VW 角速度 rad/s")
    p.add_argument("--secs", type=float, default=2.0, help="持续喷命令的秒数")
    p.add_argument("--pwm", action="store_true", help="改用 $MODE,PWM + $PWM 开环(绕开 PID/编码器)")
    args = p.parse_args()

    fd = open_uart(args.port, args.baud)
    try:
        print(f"[{args.port} @{args.baud}]")
        print("== 基线状态 ==")
        print(tx(fd, "$GET,STATUS\r\n") or "<无应答>")

        if args.pwm:
            pwm = int(args.v if abs(args.v) > 10 else 3000)  # 默认 3000 占空比
            print("== 进 PWM 开环 ==")
            print(tx(fd, "$MODE,PWM\r\n") or "<无应答>")
            frame = f"$PWM,{pwm},{pwm}\r\n"
        else:
            print("== 进 VW 模式 ==")
            print(tx(fd, "$MODE,VW\r\n") or "<无应答>")
            frame = f"$VW,{args.v},{args.w}\r\n"

        print(f"== 持续喷 {frame.strip()} 约 {args.secs}s,看轮子转不转 ==")
        n = int(args.secs / 0.05)
        for _ in range(n):
            os.write(fd, frame.encode())
            termios.tcdrain(fd)
            drain(fd, 0.02)

        print("== 运动中状态(关键:tgt/rpm/pwm/fault/bat) ==")
        print(tx(fd, "$GET,STATUS\r\n") or "<无应答>")

        print("== 停 ==")
        print(tx(fd, "$STOP\r\n") or "<无应答>")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
