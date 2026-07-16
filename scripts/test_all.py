#!/usr/bin/env python3
"""
统一测试入口 —— 一条命令跑完,自己出 ✅/❌ 总表。别贴日志,看最后那张表就行。

    python3 car/scripts/test_all.py --where local     # 开发机,不用板子
    python3 car/scripts/test_all.py --where car       # 车板
    python3 fly_car/scripts/test_all.py --where flycar    # 飞车板(先 export ROS_DOMAIN_ID=1)
    python3 car/scripts/test_all.py --where wire --fly-ip 192.168.10.171   # 真机跨机

car/scripts/ 与 fly_car/scripts/ 是同一份,改一边要同步另一边。

四个 where 各查什么:

  local   纯静态 + 单测,不需要 ROS/硬件:
            · 跨机 UDP 协议三方对齐(两侧 C++ + Python)
            · terminal 单测(规划/两批切分/起飞两步/语音状态机)

  car     local 的全部 + 车桥活体:
            · FC0E 起飞放行重发、FC0D 到达去重、FC0C pose 回归

  flycar  飞车桥 + 航点执行器活体:
            · 到达检测(把起飞点设成当前位置触发,再设 10m 外验反例)
            · FC0E → 飞车本地话题
            · **route_target_publisher 收话题航点 → /target_position**(最新的代码)

  wire    两块板真机联调(先在飞车跑 --where flycar-agent):
            · 车本地话题 →真桥→ UDP →真桥→ 飞车本地话题
            · 飞车 → UDP FC0D → 车

退出码: 0 = 全过, 1 = 有 ❌
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

# 仓库根:本文件两侧都在 <root>/<设备>/scripts/(car/scripts 与 fly_car/scripts 同一份)
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    results.append((name, ok, detail))
    print(f"  {'✅' if ok else '❌'} {name}" + (f" —— {detail}" if detail else ""))


def run(cmd: list[str], cwd: Path, timeout: int = 180) -> tuple[bool, str]:
    """跑一条命令,返回 (成功, 末尾输出)。不把子进程日志往外倒 —— 只留结论。"""
    try:
        p = subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, f"超时 {timeout}s"
    except FileNotFoundError as e:
        return False, f"命令不存在: {e}"
    tail = (p.stdout + p.stderr).strip().splitlines()
    return p.returncode == 0, (tail[-1] if tail else "")


# ══════════════════════ local:静态 + 单测 ══════════════════════

def run_local() -> None:
    print("\n【1】跨机 UDP 协议三方对齐(两侧 C++ + Python)")
    ok, tail = run([sys.executable, "car/scripts/check_udp_protocol.py"], ROOT)
    check("magic 号段/包结构对齐", ok, tail if not ok else "Pose=FC0C LaunchArrived=FC0D FlightEnable=FC0E")

    print("\n【2】C++ 语法检查(开发机 Jazzy;板上是 Humble,只当提前排错)")
    check_cpp_syntax()

    print("\n【3】新写的节点有没有 launch 起(写了没接线 = 白写)")
    check_scripts_are_launched()
    print("\n     调试脚本 / 任务脚本语法")
    check_py_syntax()

    print("\n【4】terminal 单测(规划 / 两批 / 起飞两步 / 语音状态机)")
    ok, tail = run(
        [sys.executable, "-m", "unittest", "scripts/test_l_path_tuning.py", "-v"],
        ROOT / "fly_car",
    )
    check("飞车固定 L 采集器纯单测", ok, tail)

    terminal = ROOT / "terminal"
    if not (terminal / "tests").is_dir():
        check("terminal 单测", False, "找不到 terminal/tests")
        return
    for name in sorted(p.stem for p in (terminal / "tests").glob("test_*.py")):
        ok, tail = run([sys.executable, "-m", "unittest", f"tests.{name}"], terminal)
        check(f"terminal/{name}", ok, tail)


# 每个任务脚本必须被某个 launch 起来,否则跑上板才发现"这功能怎么没反应"。
# (2026-07-15 就漏过一次:car_rescuee_drop.py 写完了,没有任何 launch 起它,
#  车侧整条投放链路是死的。)
LAUNCHED_SCRIPTS = [
    ("car_rescuee_drop.py", ["car/car_launch/launch"]),
    ("rescue_drop_sequencer.py", ["fly_car/src/my_launch/launch"]),
    ("servo_camera_by_mode.py", ["fly_car/src/my_launch/launch"]),
]

# 调试工具本身也得能跑 —— 上板时它俩是唯一的眼睛,语法错了整轮测试就瞎了。
PY_SYNTAX_FILES = [
    "car/scripts/patrol_monitor.py",
    "car/scripts/car_rescuee_drop.py",
    "car/scripts/test_rescue_handshake.py",
    "car/scripts/check_udp_protocol.py",
    "fly_car/scripts/patrol_monitor_fly.py",
    "fly_car/scripts/rescue_drop_sequencer.py",
    "fly_car/scripts/servo_camera_by_mode.py",
    "fly_car/scripts/l_path_tuning.py",
    "fly_car/scripts/test_l_path_tuning.py",
]


def check_py_syntax() -> None:
    for rel in PY_SYNTAX_FILES:
        src = ROOT / rel
        if not src.is_file():
            check(f"py {Path(rel).name}", False, f"文件不存在: {rel}")
            continue
        ok, tail = run([sys.executable, "-m", "py_compile", str(src)], ROOT, timeout=60)
        check(f"py {Path(rel).name}", ok, tail if not ok else "语法过")


def check_scripts_are_launched() -> None:
    for name, launch_dirs in LAUNCHED_SCRIPTS:
        found = ""
        for d in launch_dirs:
            for f in (ROOT / d).glob("*.launch.py"):
                if name in f.read_text(encoding="utf-8"):
                    found = f.name
                    break
            if found:
                break
        check(f"{name} 有 launch 起", bool(found),
              found if found else "没有任何 launch 起它 —— 这功能上板不会有反应")


CPP_FILES = [
    ("车侧 xmachine_bridge", "car/follower_pkg/src/xmachine_bridge.cpp", []),
    ("车侧 coverage_route_publisher", "car/follower_pkg/src/coverage_route_publisher.cpp", []),
    ("飞车侧 xmachine_bridge", "fly_car/src/activity_control_pkg/src/xmachine_bridge.cpp", []),
    ("飞车侧 route_target_publisher",
     "fly_car/src/activity_control_pkg/src/route_target_publisher.cpp",
     ["fly_car/src/activity_control_pkg/include"]),
]


def check_cpp_syntax() -> None:
    """拿开发机的 ROS 头文件给 C++ 做语法检查 —— 不编译不链接,只提前抓错。

    **不能替代板上的 colcon build**:
      · 开发机是 Jazzy(24.04),板子是 Humble(22.04),两者 API 有差异
      · -fsyntax-only 不链接,抓不到"符号未定义"
      · 不看 CMakeLists,抓不到"少了依赖"
    但拼写/类型/缺 include 这类占了编译失败的绝大多数,能在这儿先清掉。
    """
    ros_dirs = sorted(Path("/opt/ros").glob("*")) if Path("/opt/ros").is_dir() else []
    if not ros_dirs:
        check("C++ 语法检查", True, "跳过:本机没装 ROS(不算失败,板上编时才验)")
        return
    ros = ros_dirs[-1]
    includes = [f"-I{ros}/include"] + [f"-I{d}" for d in (ros / "include").glob("*/")]

    for label, rel, extra in CPP_FILES:
        src = ROOT / rel
        if not src.is_file():
            check(f"C++ {label}", False, f"文件不存在: {rel}")
            continue
        cmd = (["g++", "-fsyntax-only", "-std=c++17"] + includes
               + [f"-I{ROOT / e}" for e in extra] + [str(src)])
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            check(f"C++ {label}", False, f"跑不起来: {e}")
            continue
        errs = [ln for ln in p.stderr.splitlines() if ": error:" in ln]
        check(f"C++ {label}", not errs,
              errs[0] if errs else f"语法过({ros.name})")


# ══════════════════════ car:车桥活体 ══════════════════════

def run_car() -> None:
    run_local()
    print("\n【3】车桥活体(自拉桥走 18890/18891,假装飞车)")
    ok, tail = run([sys.executable, str(HERE / "test_rescue_handshake.py"), "--role", "car"],
                   ROOT, timeout=120)
    check("FC0E 重发 / FC0D 去重 / FC0C pose 回归", ok, tail)


# ══════════════════════ flycar:飞车桥 + 航点执行器 ══════════════════════

def run_flycar() -> None:
    if os.environ.get("ROS_DOMAIN_ID") != "1":
        print("⚠ 飞车是域1,先 export ROS_DOMAIN_ID=1(不然订不到本地话题/TF)")

    print("\n【1】飞车桥活体(到达检测 + FC0E → 本地话题)")
    ok, tail = run([sys.executable, str(HERE / "test_rescue_handshake.py"), "--role", "flycar"],
                   ROOT, timeout=120)
    check("到达触发 FC0D / 远处不触发 / FC0E 去重", ok, tail)

    print("\n【2】patrol_ground.launch.py 的外部依赖")
    check_launch_deps()

    print("\n【3】route_target_publisher 收话题航点(本次新写,最没验过)")
    test_route_topic()


def check_launch_deps() -> None:
    """launch 里 ExecuteProcess 跑的脚本是按板上绝对路径找的,少一个就静默不生效。"""
    for name in ("servo_camera_by_mode.py", "servo_set_once.py",
                 "rescue_drop_sequencer.py"):
        p = Path(os.path.expanduser(f"~/kian_flycar/scripts/{name}"))
        check(f"~/kian_flycar/scripts/{name}", p.is_file(),
              "在" if p.is_file() else "缺 —— launch 里 ExecuteProcess 会静默失败,摄像头角度不切")
    # launch 自己装没装上(setup.py 是 glob,重新 colcon build 才会进 install)
    ok, tail = run(["ros2", "launch", "my_launch", "patrol_ground.launch.py",
                    "--show-args"], ROOT, timeout=30)
    check("patrol_ground.launch.py 已装到 install", ok,
          tail if not ok else "with_video / launch_check_hz 可配")


def test_route_topic() -> None:
    """起一个 route_target_publisher,用话题喂航点,看 /target_position 跟不跟。"""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
        from std_msgs.msg import Float32MultiArray
    except ImportError as e:
        check("route_target_publisher 收航点", False, f"没有 rclpy: {e}")
        return

    ROUTE_TOPIC = "/test/route_waypoints"
    OUT_TOPIC = "/test/target_position"

    proc = subprocess.Popen(
        ["ros2", "run", "activity_control_pkg", "route_test_node", "--ros-args",
         "-p", "preload_waypoints:=false",       # 不预装,静等下发(空列表传不进参数系统)
         "-p", f"route_topic:={ROUTE_TOPIC}",
         "-p", f"output_topic:={OUT_TOPIC}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    rclpy.init()
    node = Node("route_topic_test")
    latched = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    targets: list[tuple] = []
    node.create_subscription(Float32MultiArray, OUT_TOPIC,
                             lambda m: targets.append(tuple(m.data[:4])), latched)
    route_pub = node.create_publisher(Float32MultiArray, ROUTE_TOPIC, latched)

    def spin(sec: float) -> None:
        import time
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        spin(3.0)
        # preload_waypoints:=false 时不该自己飞演示航点 —— 那条默认航点会让飞车前进 2m 再升 100cm
        check("preload_waypoints:=false 时不预装航点", not targets,
              "静等下发,对" if not targets else f"竟然自己发了目标 {targets[0]} —— 会飞出去!")

        # 第一条路线:地面段 → 原地拉高 → 转 yaw(就是 planner 的两批结构)
        targets.clear()
        m = Float32MultiArray()
        m.data = [0.0, 0.0, 0.0, 0.0, 350.0, -150.0, 0.0, 0.0]
        route_pub.publish(m)
        spin(2.5)
        check("话题下发路线 → 出 /target_position", len(targets) >= 1,
              f"首点={targets[0]}" if targets else "没反应(没重编译? route_topic 没配上?)")
        if targets:
            check("首点就是路线第一个点", targets[0] == (0.0, 0.0, 0.0, 0.0), f"{targets[0]}")

        # 整条换:第二批(空中段)来了要立刻改追新首点,不能还在追旧路线
        targets.clear()
        m2 = Float32MultiArray()
        m2.data = [350.0, -150.0, 120.0, 0.0, 350.0, -150.0, 120.0, -90.0]
        route_pub.publish(m2)
        spin(2.5)
        check("新路线整条替换旧的", bool(targets) and targets[-1][:3] == (350.0, -150.0, 120.0),
              f"当前目标={targets[-1]}" if targets else "没换过来")

        # 空路线不能把队列清掉(否则飞车飞一半没目标了)
        targets.clear()
        m3 = Float32MultiArray()
        m3.data = []
        route_pub.publish(m3)
        spin(1.5)
        check("空路线被忽略,不清队列", not targets or targets[-1][:3] == (350.0, -150.0, 120.0),
              "忽略了,对")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        import signal
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


# ══════════════════════ wire:真机跨机 ══════════════════════

def run_wire(fly_ip: str) -> None:
    print(f"\n【1】真机跨机(需飞车先跑 --where flycar-agent),飞车 IP={fly_ip}")
    ok, tail = run([sys.executable, str(HERE / "test_rescue_handshake.py"),
                    "--role", "wire", "--fly-ip", fly_ip], ROOT, timeout=120)
    check("车→UDP→飞车 / 飞车→UDP→车 / pose 心跳", ok, tail)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="统一测试入口(跨机只走裸 UDP,本地才是 DDS)",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--where", required=True,
                    choices=["local", "car", "flycar", "flycar-agent", "wire"])
    ap.add_argument("--fly-ip", default="192.168.10.171")
    args = ap.parse_args()

    if args.where == "flycar-agent":
        # 探针没有"结论",它就是挂在那儿等车侧驱动
        os.execv(sys.executable,
                 [sys.executable, str(HERE / "test_rescue_handshake.py"), "--role", "agent"])

    print("=" * 66)
    print(f" 统一测试: --where {args.where}   仓库根={ROOT}")
    print("=" * 66)

    if args.where == "local":
        run_local()
    elif args.where == "car":
        run_car()
    elif args.where == "flycar":
        run_flycar()
    else:
        run_wire(args.fly_ip)

    bad = [r for r in results if not r[1]]
    print("\n" + "=" * 66)
    print(f" 总表 (--where {args.where})")
    print("=" * 66)
    for name, ok, detail in results:
        print(f"{'✅' if ok else '❌'} {name}" + (f"\n      {detail}" if detail and not ok else ""))
    print("=" * 66)
    if bad:
        print(f"❌ {len(bad)}/{len(results)} 项没过 —— 上面 ❌ 的行就是断点")
        return 1
    print(f"✅ 全过({len(results)} 项)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
