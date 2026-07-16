#!/usr/bin/env python3
"""
跨机 UDP 桥(xmachine_bridge)诊断工具 —— 不依赖 ROS,纯标准库。

用途:把"terminal 说开始救援→车→飞车→mission 开跑"这条链路拆成几段,
逐段确认到底断在哪。包格式与两端 C++ 的 BoolPacket 完全一致:
    struct __attribute__((packed)) { uint16 magic; uint16 id; uint8 value; }  # 5 字节,小端

端口约定(来自两端代码):
    飞车 bind 8891 收车的包(FC04/FC06/FC07)
    车   bind 8890 收飞车的包(REQ/OBST/RESCUEE)

事件 magic:
    start   = 0xFC07   terminal"开始救援"→ 飞车发本地 /mission_start
    confirm = 0xFC06   放行右转投货
    done    = 0xFC04   车补给完成

⚠️ 注意:飞车端对 start 用 start_published_ 做幂等,一个 xmachine_bridge 进程
   只认第一次 START。测过一次后要重发,得先重启飞车的 demo_ground。confirm/done 同理各有 flag。

======================== 典型用法 ========================

【段1】飞车侧 UDP→/mission_start→mission 是否通(在飞车本机,demo_ground 正在跑时另开终端):
    python3 test_xmachine_bridge.py send --target 127.0.0.1 --event start
  期望飞车日志出现:
    xmachine_bridge: "收到车 UDP start,本地发 /mission_start=1"
    relief_drop_mission: "收到 /mission_start=1 ..."
  → 出现 = QoS 修复 + 飞车侧整段 OK,问题纯在"车没把包发过来"。
  → 不出现 = 飞车侧还有问题(端口/防火墙/状态机)。

【段2】车→飞车网络是否通(在车那台机上跑,打飞车真实 IP):
    python3 test_xmachine_bridge.py send --target 192.168.10.171 --event start
  → 飞车出现上面日志 = 网络通,病根在车端 xmachine_bridge 没把 ROS /mission_start 转成 UDP。
  → 飞车没反应 = 车→飞车 UDP 不通(IP 错/不同子网/防火墙)。

【段3】抓包看车端到底发没发(在飞车本机,先 Ctrl-C 停掉 demo_ground 释放 8891,再跑):
    python3 test_xmachine_bridge.py listen --port 8891
  然后在车端 terminal 说"开始救援"、或 `ros2 topic pub --once /mission_start std_msgs/msg/Bool "{data: true}"`。
  → 抓到 magic=0xFC07 = 车端确实发出来了(那飞车 bridge 没转就是 bridge 的问题)。
  → 抓不到 = 车端根本没发(车端 xmachine_bridge 没收到 terminal 的 /mission_start,或没在跑)。
"""
import argparse
import socket
import struct
import sys
import time

MAGIC = {"start": 0xFC07, "confirm": 0xFC06, "done": 0xFC04}
NAME_BY_MAGIC = {v: k for k, v in MAGIC.items()}
PACKET_FMT = "<HHB"           # magic(u16 LE), id(u16 LE), value(u8)
PACKET_SIZE = struct.calcsize(PACKET_FMT)


def do_send(args):
    magic = MAGIC[args.event]
    pkt = struct.pack(PACKET_FMT, magic, args.id, 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"→ 向 {args.target}:{args.port} 发 {args.event}(magic=0x{magic:04X} id={args.id}),"
          f"burst {args.count} 次 @ {args.hz}Hz,模拟车端抗丢包重发")
    interval = 1.0 / args.hz if args.hz > 0 else 0.0
    for i in range(args.count):
        sock.sendto(pkt, (args.target, args.port))
        if interval:
            time.sleep(interval)
    sock.close()
    print(f"✓ 已发完 {args.count} 个包。去飞车日志找 \"收到车 UDP {args.event}\" / \"收到 /mission_start=1\"。")


def do_listen(args):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", args.port))
    except OSError as e:
        print(f"✗ bind {args.port} 失败:{e}\n  多半是 xmachine_bridge 还占着这个端口,先停掉它再抓包。",
              file=sys.stderr)
        sys.exit(1)
    print(f"← 监听 0.0.0.0:{args.port},等车端的包(Ctrl-C 退出)…")
    seen = {}
    while True:
        data, addr = sock.recvfrom(2048)
        if len(data) >= PACKET_SIZE:
            magic, pid, value = struct.unpack(PACKET_FMT, data[:PACKET_SIZE])
            name = NAME_BY_MAGIC.get(magic, f"未知(0x{magic:04X})")
            key = (magic, pid)
            n = seen.get(key, 0) + 1
            seen[key] = n
            # 同一 burst 只在首包详细打印,后续计数,避免刷屏
            if n == 1:
                print(f"  收到 {name}  id={pid} value={value}  来自 {addr[0]}:{addr[1]}  ({len(data)}B)")
            else:
                print(f"  …{name} id={pid} 重发 x{n}", end="\r")
        else:
            print(f"  收到 {len(data)}B 短包(非 BoolPacket)来自 {addr[0]}")


def main():
    ap = argparse.ArgumentParser(
        description="xmachine_bridge 跨机 UDP 链路诊断(见文件头注释的分段用法)",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("send", help="模拟车端,往飞车发一个事件包")
    s.add_argument("--target", required=True, help="飞车 IP(本机测用 127.0.0.1,真机用 192.168.10.171)")
    s.add_argument("--port", type=int, default=8891, help="飞车接收端口(默认 8891)")
    s.add_argument("--event", choices=list(MAGIC), default="start", help="事件类型(默认 start)")
    s.add_argument("--id", type=int, default=1, help="事件 id(默认 1;飞车对 start 幂等,重测建议换 id 或重启 bridge)")
    s.add_argument("--count", type=int, default=30, help="burst 重发次数(默认 30,和车端 resend_count 一致)")
    s.add_argument("--hz", type=float, default=10.0, help="重发频率 Hz(默认 10)")
    s.set_defaults(func=do_send)

    l = sub.add_parser("listen", help="抓包,看车端到底有没有发过来")
    l.add_argument("--port", type=int, default=8891, help="监听端口(默认 8891;需先停掉占用它的 xmachine_bridge)")
    l.set_defaults(func=do_listen)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
