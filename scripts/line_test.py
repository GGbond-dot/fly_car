#!/usr/bin/env python3
"""line_test.py —— 让飞车地面底盘走一段【直线固定距离】,验证走直/走准。

原理:$ENC 的硬件计数器被固件 encoder_update_100ms() 每 100ms 清零,不能当里程用。
但 $STATUS 里的 rpm_l/rpm_r 每 100ms 更新,对它积分即可估算里程:
  单轮线速度 v_wheel = rpm/60 * (pi * wheel_diameter)   [m/s]
  车前进距离      += (v_left + v_right)/2 * dt
到达目标距离就 $STOP。左右轮各自里程差 => 跑偏方向/大小。

流程:$MODE,VW -> 循环{喷 $VW,v,0 + 每 100ms 读 $STATUS 积分} -> 到距离 $STOP -> 报告

用法(**先在地面留出 >1.2m 直线空间,手放电源开关上**):
  python3 line_test.py                 # 默认走 1.00m @ 0.12 m/s,/dev/ttyS3
  python3 line_test.py --dist 1.0 --v 0.12
  python3 line_test.py --dist 0.5 --v 0.15
  python3 line_test.py --diam 0.072    # 若实测轮径不同,改这里让里程更准

看结束报告:
  里程 ~1.00m 且 左右轮里程差很小(<2cm)   -> 走得直、走得准 ✔
  里程对但左右差大 / 车明显跑偏             -> 左右轮不平衡:量一下偏哪边告诉我
  里程估算和尺子实测差很多                  -> 轮径/PPR 标定不准,拿尺子实测反推 --diam
  rpm 飙升 / pwm 冲顶                        -> 立刻断电,方向或 PID 不稳
"""

import argparse
import math
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


def parse_rpm(status):
    """从 $STATUS,...,rpm_l=NN,rpm_r=NN,... 里取左右轮 rpm，取不到返回 None。"""
    if not status:
        return None
    rl = rr = None
    for tok in status.replace(",", " ").split():
        if tok.startswith("rpm_l="):
            rl = int(tok[6:])
        elif tok.startswith("rpm_r="):
            rr = int(tok[6:])
    if rl is None or rr is None:
        return None
    return rl, rr


def main():
    p = argparse.ArgumentParser(description="飞车底盘 直线固定距离 测试")
    p.add_argument("--port", default="/dev/ttyS3")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--dist", type=float, default=1.0, help="目标距离 m（默认 1.00m = 100cm）")
    p.add_argument("--v", type=float, default=0.12, help="线速度 m/s（低速更稳更准）")
    p.add_argument("--diam", type=float, default=0.072, help="轮径 m（默认与固件一致 0.072）")
    args = p.parse_args()

    circ = math.pi * args.diam            # 轮周长 m
    dt = 0.1                              # 与固件 rpm 更新周期 100ms 对齐
    tx_period = 0.05                      # 每 50ms 喷一帧 $VW，防命令超时停车
    timeout = args.dist / max(args.v, 1e-3) * 2.5 + 3.0  # 安全上限，超时强停

    fd = open_uart(args.port, args.baud)
    try:
        print(f"[{args.port} @{args.baud}]  目标 {args.dist*100:.0f}cm @ {args.v} m/s  周长 {circ*100:.1f}cm")
        print("== 基线状态 ==")
        print(tx(fd, "$GET,STATUS\r\n") or "<无应答>")
        print("== 进 VW 模式 ==")
        print(tx(fd, "$MODE,VW\r\n") or "<无应答>")

        frame = f"$VW,{args.v},0\r\n".encode()
        dist_l = dist_r = 0.0
        t0 = time.monotonic()
        last_sample = t0
        print(f"== 走直线中（喷 $VW,{args.v},0），每 100ms 采样一次 ==")
        while True:
            os.write(fd, frame)
            termios.tcdrain(fd)
            now = time.monotonic()

            if now - last_sample >= dt:
                last_sample = now
                rpm = parse_rpm(drain(fd, 0.0) or tx(fd, "$GET,STATUS\r\n", 0.05))
                if rpm:
                    rl, rr = rpm
                    dist_l += rl / 60.0 * circ * dt
                    dist_r += rr / 60.0 * circ * dt
                    avg = (dist_l + dist_r) / 2.0
                    print(f"  t={now-t0:4.1f}s  rpm=({rl:+4d},{rr:+4d})  "
                          f"里程 L={dist_l*100:5.1f} R={dist_r*100:5.1f} 平均={avg*100:5.1f}cm")
                    if avg >= args.dist:
                        print("== 到达目标距离，停 ==")
                        break

            if now - t0 > timeout:
                print(f"== 超时 {timeout:.1f}s 强制停（是否卡住/太慢？）==")
                break
            time.sleep(max(0, tx_period - (time.monotonic() - now)))

        print(tx(fd, "$STOP\r\n") or "<无应答>")
        avg = (dist_l + dist_r) / 2.0
        drift = dist_l - dist_r
        print("\n===== 结果 =====")
        print(f"  估算里程   : L={dist_l*100:.1f}cm  R={dist_r*100:.1f}cm  平均={avg*100:.1f}cm  (目标 {args.dist*100:.0f}cm)")
        print(f"  左右轮偏差 : {drift*100:+.1f}cm  (>0 左轮走多=车向右偏, <0 车向左偏)")
        print("  ↳ 拿尺子量实际走的直线长度和横向偏移，和上面对比：")
        print("     里程准 & 偏差小 -> OK；里程差多 -> 反推 --diam；偏差大 -> 左右轮不平衡")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
