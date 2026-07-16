#!/usr/bin/env python3
"""
Tier0 语音搜救握手自检 —— 一条命令跑完,自己出结论(✅/❌ 表 + 退出码)。

链路:terminal 语音确认 →(车本地 DDS)/rescue/flight_search_enable → 车桥
     →[裸 UDP FC0E]→ 飞车桥 →(飞车本地 DDS 域1)/rescue/flight_search_enable → 允许起飞
     飞车查本地 TF 到 A4B8 → 车桥 ←[裸 UDP FC0D]← 飞车桥 → /rescue/flight_launch_arrived → terminal 播报

**跨机一律裸 UDP,绝不跨机订 ROS**。本脚本的跨机探针(agent/wire)也只走 UDP,
两边的 DDS 各自留在本机(车=域0,飞车=域1),不串台。

────────────────────────── 四种角色 ──────────────────────────

【1】车侧自测(默认,不需要飞车在线):
        python3 car/scripts/test_rescue_handshake.py --role car
    自拉一个车桥走测试端口 18890/18891,假装飞车。查 FC0E 重发、FC0D 去重、FC0C pose 回归。

【2】飞车侧自测(在飞车板上,不需要车在线):
        export ROS_DOMAIN_ID=1
        python3 scripts/test_rescue_handshake.py --role flycar
    自拉一个飞车桥,把"飞行起点"临时设成飞车此刻的位置 → 到达判定应立刻触发 FC0D。
    再把目标设到 10m 外 → 不该触发。顺带查 FC0E → 本地话题。

【3】+【4】真机跨机(两块板都要,两边真桥正常在跑):
    先在飞车板:
        export ROS_DOMAIN_ID=1
        python3 scripts/test_rescue_handshake.py --role agent
    再在车板:
        python3 car/scripts/test_rescue_handshake.py --role wire --fly-ip 192.168.10.171
    车侧驱动,飞车侧当探针;探针的结果**用 UDP 回报**给车侧,车侧打总表。

退出码:0 = 全过,1 = 有 ❌
"""

import argparse
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import time

# 自测用的假端口,不碰真桥的 8890/8891
TEST_TO_CAR_PORT = 18890
TEST_FROM_CAR_PORT = 18891
# 真桥端口(wire 模式用)
REAL_TO_CAR_PORT = 8890
REAL_FROM_CAR_PORT = 8891
AGENT_PORT = 18899          # 跨机探针的控制口(测试脚本之间,不是桥)

MAGIC_FLIGHT_ENABLE = 0xFC0E
MAGIC_LAUNCH_ARRIVED = 0xFC0D
MAGIC_POSE = 0xFC0C

BOOL_FMT = "<HHB"    # magic, id, value
POSE_FMT = "<HHff"   # magic, id, x_cm, y_cm

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    results.append((name, ok, detail))
    print(f"  {'✅' if ok else '❌'} {name}" + (f" —— {detail}" if detail else ""))


def verdict(title: str) -> int:
    bad = [r for r in results if not r[1]]
    print("\n" + "=" * 64)
    print(f" {title}")
    print("=" * 64)
    for name, ok, detail in results:
        print(f"{'✅' if ok else '❌'} {name}" + (f"  ({detail})" if detail else ""))
    print("=" * 64)
    if bad:
        print(f"❌ {len(bad)}/{len(results)} 项没过 —— 上面 ❌ 的行就是断点")
        return 1
    print(f"✅ 全过({len(results)} 项)")
    return 0


def need_ros():
    try:
        import rclpy  # noqa: F401
    except ImportError as e:
        print(f"❌ 没有 rclpy:{e}\n   先 source /opt/ros/humble/setup.bash 和 install/setup.bash")
        sys.exit(1)


def spin_for(node, seconds: float) -> None:
    import rclpy
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def start_bridge(pkg: str, exe: str, params: list[str]):
    return subprocess.Popen(
        ["ros2", "run", pkg, exe, "--ros-args"] + params,
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid)


def stop_bridge(proc) -> None:
    if proc is None:
        return
    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


def bind_udp(port: int, what: str):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("0.0.0.0", port))
    except OSError as e:
        print(f"❌ bind {port} 失败({what}):{e}\n   上次没退干净? pkill -f test_rescue_handshake")
        sys.exit(1)
    s.settimeout(0.2)
    return s


def drain_bool(sock, seconds: float) -> list[tuple[int, int]]:
    """收 seconds 秒内的 BoolPacket,返回 [(magic,id)]"""
    got = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        if len(data) == struct.calcsize(BOOL_FMT):
            magic, pid, val = struct.unpack(BOOL_FMT, data)
            if val:
                got.append((magic, pid))
    return got


# ══════════════════════════ 角色 1:车侧自测 ══════════════════════════

def role_car() -> int:
    need_ros()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from std_msgs.msg import Bool, Float32MultiArray

    fly = bind_udp(TEST_FROM_CAR_PORT, "假装飞车收车的包")
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("拉起车桥(测试端口 18890/18891,fly_ip=127.0.0.1)...")
    bridge = start_bridge("follower_pkg", "xmachine_bridge", [
        "-p", "fly_ip:=127.0.0.1",
        "-p", f"to_car_port:={TEST_TO_CAR_PORT}",
        "-p", f"from_car_port:={TEST_FROM_CAR_PORT}",
        "-p", "resend_count:=30", "-p", "send_hz:=10.0"])

    rclpy.init()
    node = Node("rescue_handshake_car_test")
    ev_qos = QoSProfile(depth=10,
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.VOLATILE)
    latched = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    arrived: list[bool] = []
    poses: list[tuple] = []
    node.create_subscription(Bool, "/rescue/flight_launch_arrived",
                             lambda m: arrived.append(m.data), ev_qos)
    node.create_subscription(Float32MultiArray, "/flycar/pose",
                             lambda m: poses.append(tuple(m.data)), latched)
    enable_pub = node.create_publisher(Bool, "/rescue/flight_search_enable", ev_qos)

    try:
        print("等桥起来 + 话题发现(3s)...")
        spin_for(node, 3.0)
        drain_bool(fly, 0.3)

        print("\n[1] 车本地 /rescue/flight_search_enable → 裸 UDP FC0E 发飞车")
        m = Bool(); m.data = True
        enable_pub.publish(m)
        spin_for(node, 0.2)
        pkts = drain_bool(fly, 4.5)
        fc0e = [p for p in pkts if p[0] == MAGIC_FLIGHT_ENABLE]
        check("FC0E 发出去了", len(fc0e) > 0,
              f"收到 {len(fc0e)} 个" if fc0e else "一个都没收到(桥没订上? 没重编译?)")
        if fc0e:
            check("FC0E 按 resend_count 重发", len(fc0e) >= 25, f"{len(fc0e)}/30 个(抗丢包)")
            check("FC0E 同一事件同一 id", len({p[1] for p in fc0e}) == 1, f"id={fc0e[0][1]}")
        check("起飞放行没发成 FC0C", not [p for p in pkts if p[0] == MAGIC_POSE],
              "FC0C 是 pose 的号!" if [p for p in pkts if p[0] == MAGIC_POSE] else "用的 FC0E,对")

        print("\n[2] 飞车 FC0D(burst 30 包)→ 车桥去重 → 只发布一次")
        arrived.clear()
        pkt = struct.pack(BOOL_FMT, MAGIC_LAUNCH_ARRIVED, 7, 1)
        for _ in range(30):
            tx.sendto(pkt, ("127.0.0.1", TEST_TO_CAR_PORT))
            time.sleep(0.01)
        spin_for(node, 1.5)
        check("FC0D 发布了 /rescue/flight_launch_arrived", len(arrived) >= 1, f"{len(arrived)} 次")
        check("FC0D 30 个重复包只发布一次", len(arrived) == 1,
              "去重生效" if len(arrived) == 1 else f"发布 {len(arrived)} 次,terminal 会重复播报")

        print("\n[3] FC0D 换 id(新事件不能被去重吃掉)")
        arrived.clear()
        for _ in range(5):
            tx.sendto(struct.pack(BOOL_FMT, MAGIC_LAUNCH_ARRIVED, 8, 1),
                      ("127.0.0.1", TEST_TO_CAR_PORT))
            time.sleep(0.01)
        spin_for(node, 1.5)
        check("新 id 能再次发布", len(arrived) == 1, f"{len(arrived)} 次")

        print("\n[4] 回归:FC0C → /flycar/pose(点云逐步露出靠它,合并时差点被删)")
        poses.clear()
        for i, (x, y) in enumerate([(100.0, -50.0), (150.0, -60.0)], start=1):
            tx.sendto(struct.pack(POSE_FMT, MAGIC_POSE, i, x, y), ("127.0.0.1", TEST_TO_CAR_PORT))
            time.sleep(0.15)
        spin_for(node, 1.5)
        check("FC0C 还在发 /flycar/pose", len(poses) >= 1,
              f"{len(poses)} 个位置" if poses else "pose 链路被合并弄丢了!")
        if poses:
            check("pose 数值对得上", poses[-1] == (150.0, -60.0), f"最后一个={poses[-1]}")
    finally:
        node.destroy_node(); rclpy.shutdown(); stop_bridge(bridge)
        fly.close(); tx.close()
    return verdict("车侧自测(本机,假装飞车)")


# ══════════════════════════ 角色 2:飞车侧自测 ══════════════════════════

def lookup_tf(node, timeout_s: float = 8.0):
    """本地查 map->laser_link(飞车本机 DDS,不跨机)。返回 (x_m,y_m) 或 None"""
    import rclpy
    from tf2_ros import Buffer, TransformListener
    buf = Buffer()
    TransformListener(buf, node)
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            tf = buf.lookup_transform("map", "laser_link", rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            continue
    return None


def role_flycar() -> int:
    need_ros()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from std_msgs.msg import Bool

    if os.environ.get("ROS_DOMAIN_ID") != "1":
        print("⚠ 飞车是域1,先 export ROS_DOMAIN_ID=1 再跑(不然查不到 TF)")

    rclpy.init()
    node = Node("rescue_handshake_flycar_test")
    print("本地查 TF map->laser_link(拿飞车此刻位置)...")
    pose = lookup_tf(node)
    if pose is None:
        print("❌ 8 秒拿不到 TF map->laser_link —— carto 没起? 到达检测没法测。")
        node.destroy_node(); rclpy.shutdown()
        return 1
    print(f"   飞车当前位置 = ({pose[0]:.2f}, {pose[1]:.2f}) m")

    car = bind_udp(TEST_TO_CAR_PORT, "假装车收飞车的包")
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ev_qos = QoSProfile(depth=10,
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.VOLATILE)
    enabled: list[bool] = []
    node.create_subscription(Bool, "/rescue/flight_search_enable",
                             lambda m: enabled.append(m.data), ev_qos)

    # ---- 正例:把"飞行起点"设成飞车此刻的位置 → 应立刻判定到达 ----
    print("\n[1] 到达判定(正例):把 A4B8 临时设成飞车当前位置 → 该触发 FC0D")
    bridge = start_bridge("activity_control_pkg", "xmachine_bridge", [
        "-p", "car_ip:=127.0.0.1",
        "-p", f"to_car_port:={TEST_TO_CAR_PORT}",
        "-p", f"from_car_port:={TEST_FROM_CAR_PORT}",
        "-p", f"launch_target_x_m:={pose[0]}",
        "-p", f"launch_target_y_m:={pose[1]}",
        "-p", "launch_tol_m:=0.30", "-p", "launch_stable_s:=0.5",
        "-p", "pose_hz:=0.0", "-p", "resend_count:=30", "-p", "send_hz:=10.0"])
    try:
        spin_for(node, 3.0)
        pkts = drain_bool(car, 5.0)
        fc0d = [p for p in pkts if p[0] == MAGIC_LAUNCH_ARRIVED]
        check("到了就发 FC0D", len(fc0d) > 0,
              f"收到 {len(fc0d)} 个" if fc0d else "没触发(TF 太旧? 容差/稳定窗?)")
        if fc0d:
            check("FC0D 按 resend_count 重发", len(fc0d) >= 25, f"{len(fc0d)}/30 个")
            check("FC0D 同一事件同一 id", len({p[1] for p in fc0d}) == 1, f"id={fc0d[0][1]}")

        print("\n[2] FC0E → 飞车本地(域1 DDS)/rescue/flight_search_enable")
        enabled.clear()
        for _ in range(30):     # 模拟车桥 burst
            tx.sendto(struct.pack(BOOL_FMT, MAGIC_FLIGHT_ENABLE, 3, 1),
                      ("127.0.0.1", TEST_FROM_CAR_PORT))
            time.sleep(0.01)
        spin_for(node, 1.5)
        check("FC0E 发了本地 /rescue/flight_search_enable", len(enabled) >= 1, f"{len(enabled)} 次")
        check("FC0E 30 个重复包只发一次", len(enabled) == 1,
              "去重生效" if len(enabled) == 1 else f"发了 {len(enabled)} 次")
    finally:
        stop_bridge(bridge)

    # ---- 反例:目标挪到 10m 外 → 不该触发 ----
    print("\n[3] 到达判定(反例):A4B8 设到 10m 外 → 不该发 FC0D")
    drain_bool(car, 0.3)
    bridge2 = start_bridge("activity_control_pkg", "xmachine_bridge", [
        "-p", "car_ip:=127.0.0.1",
        "-p", f"to_car_port:={TEST_TO_CAR_PORT}",
        "-p", f"from_car_port:={TEST_FROM_CAR_PORT}",
        "-p", f"launch_target_x_m:={pose[0] + 10.0}",
        "-p", f"launch_target_y_m:={pose[1] + 10.0}",
        "-p", "launch_tol_m:=0.15", "-p", "pose_hz:=0.0"])
    try:
        spin_for(node, 4.0)
        far = [p for p in drain_bool(car, 2.0) if p[0] == MAGIC_LAUNCH_ARRIVED]
        check("没到就不发 FC0D", not far,
              "没发,对" if not far else f"离 14m 还报到达({len(far)} 包)!容差判反了?")
    finally:
        stop_bridge(bridge2)
        node.destroy_node(); rclpy.shutdown(); car.close(); tx.close()
    return verdict("飞车侧自测(本机,假装车)")


# ══════════════════════════ 角色 3:飞车探针(跨机用) ══════════════════════════

def role_agent() -> int:
    """在飞车板上跑。只用 UDP 跟车侧测试脚本讲话;DDS 只在飞车本机(域1)用。"""
    need_ros()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from std_msgs.msg import Bool

    if os.environ.get("ROS_DOMAIN_ID") != "1":
        print("⚠ 飞车是域1,先 export ROS_DOMAIN_ID=1 再跑(不然订不到飞车本地话题)")

    rclpy.init()
    node = Node("rescue_handshake_agent")
    ev_qos = QoSProfile(depth=10,
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.VOLATILE)
    enabled: list[float] = []
    # 飞车本地话题(域1 DDS)—— 车侧看不到它(两机 DDS 隔离),所以才要这个探针。
    # 桥那边是 latched 发的,volatile 订能收(发布端 offer 更强,兼容)。
    node.create_subscription(Bool, "/rescue/flight_search_enable",
                             lambda m: enabled.append(time.time()), ev_qos)

    ctl = bind_udp(AGENT_PORT, "探针控制口")
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"探针就绪:UDP :{AGENT_PORT} 听车侧指令;本地(域1)订 /rescue/flight_search_enable。")
    print("等车侧跑 --role wire ... (Ctrl-C 退出)")
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                data, addr = ctl.recvfrom(4096)
            except socket.timeout:
                continue
            try:
                req = json.loads(data.decode())
            except Exception:
                continue
            cmd = req.get("cmd")
            if cmd == "ping":
                tx.sendto(json.dumps({"ok": True, "role": "agent"}).encode(), addr)
                print("← ping")
            elif cmd == "reset":
                enabled.clear()
                tx.sendto(json.dumps({"ok": True}).encode(), addr)
                print("← reset")
            elif cmd == "enabled_count":
                spin_for(node, 0.3)
                tx.sendto(json.dumps({"ok": True, "count": len(enabled)}).encode(), addr)
                print(f"← enabled_count = {len(enabled)}")
            elif cmd == "send_fc0d":
                # 模拟飞车桥往车发到达(真到达判定归 --role flycar 测)
                car_ip = req.get("car_ip"); eid = int(req.get("id", 11))
                pkt = struct.pack(BOOL_FMT, MAGIC_LAUNCH_ARRIVED, eid, 1)
                for _ in range(30):
                    tx.sendto(pkt, (car_ip, REAL_TO_CAR_PORT))
                    time.sleep(0.01)
                tx.sendto(json.dumps({"ok": True}).encode(), addr)
                print(f"← send_fc0d → {car_ip}:{REAL_TO_CAR_PORT} (id={eid})")
            elif cmd == "bye":
                tx.sendto(json.dumps({"ok": True}).encode(), addr)
                print("← bye")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown(); ctl.close(); tx.close()
    return 0


# ══════════════════════════ 角色 4:真机跨机(车侧驱动) ══════════════════════════

def role_wire(fly_ip: str) -> int:
    need_ros()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from std_msgs.msg import Bool, Float32MultiArray

    ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctl.settimeout(5.0)

    def ask(cmd: str, **kw):
        try:
            ctl.sendto(json.dumps({"cmd": cmd, **kw}).encode(), (fly_ip, AGENT_PORT))
            data, _ = ctl.recvfrom(4096)
            return json.loads(data.decode())
        except (socket.timeout, OSError, ValueError):
            return None

    print(f"联系飞车探针 {fly_ip}:{AGENT_PORT} ...")
    pong = ask("ping")
    check("飞车探针在线(UDP 通)", pong is not None and pong.get("ok"),
          "飞车没跑 --role agent? 或网不通/防火墙" if not pong else f"{fly_ip}")
    if not pong:
        return verdict("真机跨机(车侧驱动)")

    rclpy.init()
    node = Node("rescue_handshake_wire_test")
    ev_qos = QoSProfile(depth=10,
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.VOLATILE)
    latched = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    arrived: list[bool] = []
    poses: list[tuple] = []
    node.create_subscription(Bool, "/rescue/flight_launch_arrived",
                             lambda m: arrived.append(m.data), ev_qos)
    node.create_subscription(Float32MultiArray, "/flycar/pose",
                             lambda m: poses.append(tuple(m.data)), latched)
    enable_pub = node.create_publisher(Bool, "/rescue/flight_search_enable", ev_qos)
    spin_for(node, 2.0)

    try:
        # ---- 车 → 飞车:真桥 + 真网 + 真 FC0E ----
        print("\n[1] 车本地话题 →真车桥→ 裸 UDP FC0E →真飞车桥→ 飞车本地话题")
        ask("reset")
        m = Bool(); m.data = True
        enable_pub.publish(m)
        spin_for(node, 5.0)     # 车桥 burst 30 包 @10Hz ≈ 3s
        rep = ask("enabled_count")
        got = (rep or {}).get("count", 0)
        check("飞车本地收到 /rescue/flight_search_enable", got >= 1,
              f"探针报 {got} 次" if rep else "探针没应答")
        check("跨机 burst 后飞车只发一次(按 id 去重)", got == 1,
              "去重生效" if got == 1 else f"{got} 次 —— 飞车桥没按 id 去重?")

        # ---- 飞车 → 车:真 FC0D 过网 ----
        print("\n[2] 飞车 →裸 UDP FC0D→ 真车桥 → /rescue/flight_launch_arrived")
        arrived.clear()
        ok = ask("send_fc0d", car_ip=local_ip_toward(fly_ip), id=21)
        spin_for(node, 3.0)
        check("跨机 FC0D 到车并发布", len(arrived) >= 1,
              f"{len(arrived)} 次" if ok else "探针发包失败")
        check("跨机 FC0D burst 只发布一次", len(arrived) == 1,
              "去重生效" if len(arrived) == 1 else f"{len(arrived)} 次")

        # ---- pose:真桥 1Hz 心跳,顺带证明 飞车→车 这条线一直活着 ----
        print("\n[3] 回归:真飞车桥的 FC0C pose(1Hz)有没有过来")
        poses.clear()
        spin_for(node, 4.0)
        check("跨机 FC0C pose 在回传", len(poses) >= 1,
              f"{len(poses)} 个位置" if poses else "飞车 carto 没起/pose_hz=0/桥没重编译")
    finally:
        ask("bye")
        node.destroy_node(); rclpy.shutdown(); ctl.close()
    return verdict("真机跨机(车侧驱动,跨机只用 UDP)")


def local_ip_toward(peer_ip: str) -> str:
    """拿本机在这条路由上的 IP(告诉飞车往哪发 FC0D)。不发包,只查路由。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((peer_ip, 9))
        return s.getsockname()[0]
    finally:
        s.close()


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Tier0 语音搜救握手自检(跨机只走裸 UDP)",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--role", choices=["car", "flycar", "agent", "wire"], default="car",
                    help="car=车侧自测(默认) flycar=飞车侧自测 agent=飞车探针 wire=真机跨机")
    ap.add_argument("--fly-ip", default="192.168.10.171", help="wire 模式:飞车 IP")
    args = ap.parse_args()
    if args.role == "car":
        return role_car()
    if args.role == "flycar":
        return role_flycar()
    if args.role == "agent":
        return role_agent()
    return role_wire(args.fly_ip)


if __name__ == "__main__":
    sys.exit(main())
