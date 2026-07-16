#!/usr/bin/env python3
"""陆空协同搜救 —— 飞车侧测试观察器(车侧对应 car/scripts/patrol_monitor.py)。

飞车板 launch 的日志被 carto/雷达/PID 刷屏,看不清任务走到哪。本脚本只订任务相关话题,
打成干净时间线,**Ctrl-C 时直接出一张结论表**。

**为什么车侧那个不够**:两机 DDS 隔离(车=域0、飞车=域1),车侧监视器看不到飞车本地的
/target_position、/flight_enable、/terminal_confirm 这些 —— 而"飞车到底动没动、
按地面还是飞行模式动、投放中断有没有跑起来"全在这些话题里。跨机只有裸 UDP,
车那边只看得到 UDP 送过去的结果。

跑法(飞车板):
    export ROS_DOMAIN_ID=1
    python3 ~/kian_flycar/fly_car/scripts/patrol_monitor_fly.py

    --launch          顺带把 my_launch/patrol_ground.launch.py 拉起来(日志丢文件)
    --launch-arg K:=V 透传给 launch(可重复,如 with_video:=false)
    --log FILE        launch 日志落到哪

看什么:
    [航点] 收到路线(第一批地面段 / 第二批空中段)/ 插队(投放中断)
    [模式] chassis_mux 按目标 z 切地面/飞行 —— 地空互斥,同时亮 = 打架
    [目标] /target_position 推进
    [投放] 收到确认 → 降 50 → 投货 → 升回
    [起飞] 放行信号(FC0E)

⚠ 常见坑:
  · preload_waypoints:=false 没传 → 飞车按内置演示航点(前进2m升100cm飞方形)自己飞出去
  · 旧剧本(relief_drop/mission_sequencer)跟本 launch 同时起 → 抢 /target_position
退出码: 0 = 全过,1 = 有 ❌
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

Z_THRESHOLD_CM = 20.0    # 与 chassis_mux 的 z_threshold_cm 一致:>20 判空中


class FlyPatrolMonitor(Node):
    def __init__(self) -> None:
        super().__init__("patrol_monitor_fly")
        self.t0 = time.monotonic()

        self.checks: dict[str, tuple[bool, str] | None] = {
            "收到第一批(地面段)": None,
            "地面段全 z=0(确认前不起飞)": None,
            "第一批到了就走地面模式": None,
            "收到起飞放行(FC0E)": None,
            "收到第二批(空中段)": None,
            "起飞拆两步(拉高→转yaw)": None,
            "起飞时切飞行模式": None,
            "地空互斥(不同时亮)": (True, "没见过同时亮,符合预期"),
            "投放:收到确认": None,
            "投放:插队降到 50cm": None,
            "投放:升回原高度": None,
            "没按演示航点乱飞": None,
            "旧剧本没在跑": None,
        }
        self.untested_hint = {
            "收到第一批(地面段)": "车那边下发了吗?(车板喊「开始搜救」或点「发送航点」)"
                            "或桥没起/UDP 不通(跑 test_all.py --where wire)",
            "地面段全 z=0(确认前不起飞)": "没收到第一批,先看上一行",
            "第一批到了就走地面模式": "chassis_mux 没起?或 route_target_publisher 没发 "
                              "/target_position(preload_waypoints:=false 传了吗?)",
            "收到起飞放行(FC0E)": "飞车到 A4B8 了吗?到了要在车那边答「是的」",
            "收到第二批(空中段)": "没确认起飞,或 terminal 下发失败",
            "起飞拆两步(拉高→转yaw)": "没收到第二批,先看上一行",
            "起飞时切飞行模式": "没收到第二批,或 chassis_mux 没按 z 切",
            "投放:收到确认": "飞到难民点了吗?rescuee_check_hz>0 吗?到了要答「需要」",
            "投放:插队降到 50cm": "rescue_drop_sequencer 没起?(脚本在 ~/kian_flycar/scripts/ 吗)",
            "投放:升回原高度": "降下去了但没升回来 —— 看 drop_sequencer 日志",
            "没按演示航点乱飞": "自检没跑到(监视器活不到 18 秒?运动链 12s 才起,别急着 Ctrl-C)",
            "旧剧本没在跑": "自检没跑到(监视器活不到 18 秒?)",
        }

        self.wp_batches = 0
        self.first_target: tuple[float, ...] | None = None
        self.last_target: tuple[float, ...] | None = None
        self.target_count = 0
        self.ground_on = False
        self.flight_on = False
        self.cruise_z: float | None = None
        self.drop_confirm_t: float | None = None
        self.saw_descend = False

        self.create_subscription(
            Float32MultiArray, "/wildlife/waypoints", self.on_waypoints, LATCHED)
        self.create_subscription(
            Float32MultiArray, "/route/insert_waypoints", self.on_insert, 10)
        self.create_subscription(
            Float32MultiArray, "/target_position", self.on_target,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, "/ground_enable", self.on_ground, LATCHED)
        self.create_subscription(Bool, "/flight_enable", self.on_flight, LATCHED)
        self.create_subscription(
            Bool, "/rescue/flight_search_enable", self.on_flight_enable_sig, LATCHED)
        self.create_subscription(Bool, "/terminal_confirm", self.on_drop_confirm, LATCHED)
        self.create_subscription(Bool, "/rescue/drop_done", self.on_drop_done, LATCHED)
        self.create_subscription(Bool, "/mission_start", self.on_mission_start, LATCHED)

        # 自检分两段:桥不依赖 TF,launch 里立刻起,3s 查得到;而 route_target_publisher /
        # chassis_mux / rescue_drop_sequencer 挂在 patrol_ground.launch.py 的
        # TimerAction(period=12.0) 后面(等 carto 出 TF),3s 查必然全部误报 —— 07-16 就是
        # 被这个假警告吓得 3 秒 Ctrl-C,而车侧 78s 才下发,飞车早没了。留足余量到 18s。
        self.env_timer = self.create_timer(3.0, self.env_check_early)
        self.late_timer = self.create_timer(18.0, self.env_check_late)
        self.say("监听中(飞车侧)—— 等车那边下发航点")
        self.say("     运动链挂在 launch 的 12s 定时器后,~18s 才起齐 —— 别急着 Ctrl-C")

    # ---------- 工具 ----------
    def stamp(self) -> str:
        return f"[{time.monotonic() - self.t0:6.1f}s]"

    def say(self, text: str) -> None:
        print(f"{self.stamp()} {text}", flush=True)

    def env_check_early(self) -> None:
        """3s:只查不依赖 TF、launch 里立刻起的东西。"""
        self.env_timer.cancel()
        if "xmachine_bridge" not in self.get_node_names():
            self.say("[自检] ⚠ xmachine_bridge 没在跑 —— 跨机桥 —— 没它收不到车发的航点")

    def env_check_late(self) -> None:
        """18s:运动链(launch 12s 定时器后才起)起齐了再查,早查必然误报。"""
        self.late_timer.cancel()
        names = self.get_node_names()
        for want, why in (
            # ⚠ 节点名是 route_test_node(launch 的 name= 和 main 里的 rclcpp::Node 都是它),
            #   route_target_publisher 只是源文件名 —— 查错名字 = 每次必报假警告(07-16 踩的)
            ("route_test_node", "航点执行器 —— 没它飞车不会动"),
            ("chassis_mux", "地空仲裁 —— 没它地面/飞控会打架"),
            ("rescue_drop_sequencer", "投放中断 —— 没它确认投放不会有反应"),
        ):
            if want not in names:
                self.say(f"[自检] ⚠ {want} 没在跑 —— {why}")

        # 旧剧本会自己往 /target_position 发写死航点,跟规划的路线抢方向盘
        old = [n for n in ("relief_drop_mission", "mission_sequencer",
                           "coverage_mission") if n in names]
        self.checks["旧剧本没在跑"] = (
            not old,
            "没起,符合预期" if not old
            else f"⚠⚠ {', '.join(old)} 在跑 —— 会跟规划的路线抢 /target_position,飞车会抽")
        if old:
            self.say(f"[自检] ❌ 旧剧本在跑: {', '.join(old)} —— 别跟本 launch 同时起!")

        # preload_waypoints:=false 没传的话,route_test_node 会按内置演示航点(200,0,4)→(200,0,100)
        # 飞方形。特征:没收到任何 /wildlife/waypoints 却已经在发目标。
        if self.wp_batches == 0 and self.first_target is not None:
            self.checks["没按演示航点乱飞"] = (
                False,
                f"⚠⚠ 没收到下发就在追目标 {[round(v, 1) for v in self.first_target]} —— "
                f"preload_waypoints:=false 没传?它要按演示航点飞方形了!")
            self.say("[自检] ❌❌ 没收到航点却已在追目标 —— 赶紧停,preload_waypoints:=false 没传!")
        else:
            self.checks["没按演示航点乱飞"] = (True, "没收到下发前不动,对")

    def dumpWaypoints(self, d: list, n: int) -> None:
        """整条路线列出来 —— 上地面前用眼睛过一遍,比什么自动检查都靠谱。

        ⚠ 这里打的是**桥换算后的 map 系**(飞车摆成开机朝场地 -y,桥把场地系转了 +90°)。
        车侧观察器打的是换算前的场地系,两边对着看:
          场地 (0,0) yaw=-90  →  map (0,0) yaw=0   ← yaw 归零 = 旋转补偿生效,
                                                      开跑前那个原地转 90° 没了
        """
        self.say("[航点]    全部航点(map 系 = 桥换算后,x_cm y_cm z_cm yaw°):")
        for i in range(n):
            x, y, z, yaw = d[4 * i:4 * i + 4]
            self.say(f"[航点]      #{i + 1:<2d} ({x:7.1f},{y:7.1f},{z:6.1f}) yaw={yaw:7.1f}")

    # ---------- 航点 ----------
    def on_waypoints(self, msg: Float32MultiArray) -> None:
        d = list(msg.data)
        n = len(d) // 4
        if n == 0:
            return
        zs = [d[i + 2] for i in range(0, len(d) - 3, 4)]
        airborne = zs[0] > Z_THRESHOLD_CM
        self.wp_batches += 1

        if not airborne:
            self.say(f"[航点] ✅ 第一批(地面段): {n} 个,首点 {[round(v, 1) for v in d[:4]]}")
            self.dumpWaypoints(d, n)
            self.checks["收到第一批(地面段)"] = (True, f"{n} 个航点")
            allzero = all(z == 0.0 for z in zs)
            self.checks["地面段全 z=0(确认前不起飞)"] = (
                allzero, "z 全 0,飞车开到起飞点会停住" if allzero
                else f"⚠ 混进空中航点 z={sorted(set(zs))} —— 确认前就会飞!")
            return

        self.say(f"[航点] ✅ 第二批(空中段): {n} 个,首点 {[round(v, 1) for v in d[:4]]}")
        self.dumpWaypoints(d, n)
        self.checks["收到第二批(空中段)"] = (True, f"{n} 个航点,首点 z={d[2]:.0f}cm")
        if n >= 2:
            p0, p1 = d[0:4], d[4:8]
            ok = (abs(p0[0] - p1[0]) < 1 and abs(p0[1] - p1[1]) < 1
                  and abs(p0[2] - p1[2]) < 1 and abs(p0[3] - p1[3]) > 1)
            self.checks["起飞拆两步(拉高→转yaw)"] = (
                ok, f"① 原地拉高到 {p0[2]:.0f}cm → ② 原地转 yaw {p0[3]:.0f}→{p1[3]:.0f}"
                if ok else f"⚠ 前两点不是「原地拉高→原地转yaw」,会边爬升边转身")

    def on_insert(self, msg: Float32MultiArray) -> None:
        d = list(msg.data)
        if len(d) < 4:
            return
        self.say(f"[投放] 插队航点: {[round(v, 1) for v in d[:4]]}(做完接着走原路线)")
        if len(d) >= 8:                      # 插的不止一个点(降→投→升)就全列出来
            self.dumpWaypoints(d, len(d) // 4)
        z = d[2]
        if self.cruise_z is not None and z < self.cruise_z - 10:
            self.saw_descend = True
            self.checks["投放:插队降到 50cm"] = (
                True, f"从 {self.cruise_z:.0f}cm 降到 {z:.0f}cm")
        elif self.saw_descend and self.cruise_z is not None and z >= self.cruise_z - 10:
            self.checks["投放:升回原高度"] = (True, f"升回 {z:.0f}cm")

    def on_target(self, msg: Float32MultiArray) -> None:
        d = tuple(msg.data[:4])
        if len(d) < 4 or d == self.last_target:
            return
        if self.first_target is None:
            self.first_target = d
        self.last_target = d
        self.target_count += 1
        z = d[2]
        # 巡航高度 = 空中段里见过的最高目标,投放降回来时拿它比
        if z > Z_THRESHOLD_CM and (self.cruise_z is None or z > self.cruise_z):
            self.cruise_z = z
        self.say(f"[目标] #{self.target_count} → x={d[0]:.0f} y={d[1]:.0f} "
                 f"z={d[2]:.0f} yaw={d[3]:.0f}  ({'空中' if z > Z_THRESHOLD_CM else '地面'})")

    # ---------- 地空模式 ----------
    def on_ground(self, msg: Bool) -> None:
        if msg.data != self.ground_on:
            self.say(f"[模式] /ground_enable = {msg.data}")
        self.ground_on = msg.data
        if msg.data and self.wp_batches >= 1 and self.checks["第一批到了就走地面模式"] is None:
            self.checks["第一批到了就走地面模式"] = (True, "z≤20 → 地面底盘,对")
        self.check_mutex()

    def on_flight(self, msg: Bool) -> None:
        if msg.data != self.flight_on:
            self.say(f"[模式] /flight_enable = {msg.data}"
                     f"{'  ← 起飞了' if msg.data else '  ← 回地面态'}")
        self.flight_on = msg.data
        if msg.data and self.checks["起飞时切飞行模式"] is None:
            self.checks["起飞时切飞行模式"] = (True, "目标 z>20 → 飞控接管,对")
        self.check_mutex()

    def check_mutex(self) -> None:
        if self.ground_on and self.flight_on:
            self.say("[模式] ❌❌ /ground_enable 和 /flight_enable 同时为真 —— "
                     "地面底盘和飞控会同时发速度,飞车会抽!")
            self.checks["地空互斥(不同时亮)"] = (
                False, "⚠⚠ 两个同时亮过 —— chassis_mux 仲裁失效")

    # ---------- 投放 / 起飞 ----------
    def on_flight_enable_sig(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.say("[起飞] ✅ 收到放行(FC0E)—— terminal 确认起飞了")
        self.checks["收到起飞放行(FC0E)"] = (True, "车那边确认了")

    def on_drop_confirm(self, msg: Bool) -> None:
        if not msg.data or self.drop_confirm_t is not None:
            return
        self.drop_confirm_t = time.monotonic()
        self.say("[投放] ✅ 收到确认投放(FC06 → /terminal_confirm)—— 该降到 50cm 投货了")
        self.checks["投放:收到确认"] = (True, "等 drop_sequencer 插队降高度")

    def on_drop_done(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.say("[投放] ✅ 投放中断完成,继续原巡航路线")
        if self.checks["投放:升回原高度"] is None:
            self.checks["投放:升回原高度"] = (True, "drop_sequencer 报完成")

    def on_mission_start(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.say("[⚠] 收到 /mission_start —— 老投放剧本的开关。本 launch 不该有人发它")

    # ---------- 结论 ----------
    def report(self) -> bool:
        print("\n" + "=" * 64)
        print("  飞车侧测试结论")
        print("=" * 64)
        failed = untested = 0
        for name, outcome in self.checks.items():
            if outcome is None:
                mark, detail = "⬜ 未测到", self.untested_hint.get(name, "没发生")
                untested += 1
            elif outcome[0]:
                mark, detail = "✅ 通过", outcome[1]
            else:
                mark, detail = "❌ 失败", outcome[1]
                failed += 1
            print(f"  {mark}  {name:<24} {detail}")
        print("=" * 64)
        if failed:
            print(f"  {failed} 项失败 —— 看上面 ❌ 那几行")
        elif untested:
            print(f"  {untested} 项没测到(⬜),其余通过")
        else:
            print("  全部通过 ✅")
        return failed == 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="飞车侧巡航/投放观察器",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--launch", action="store_true",
                    help="顺带拉起 my_launch/patrol_ground.launch.py")
    ap.add_argument("--launch-arg", action="append", default=[],
                    help="透传给 launch(可重复),如 with_video:=false")
    ap.add_argument("--log", default=os.path.expanduser("~/.ros/patrol_ground.log"))
    args = ap.parse_args()

    if os.environ.get("ROS_DOMAIN_ID") != "1":
        print("⚠ 飞车是域1,先 export ROS_DOMAIN_ID=1,否则订不到本地话题")

    proc = None
    if args.launch:
        os.makedirs(os.path.dirname(args.log), exist_ok=True)
        log = open(args.log, "w")
        print(f"--- launch 日志: {args.log}")
        proc = subprocess.Popen(
            ["ros2", "launch", "my_launch", "patrol_ground.launch.py", *args.launch_arg],
            stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    rclpy.init()
    node = FlyPatrolMonitor()
    ok = True
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        ok = node.report()
        node.destroy_node()
        # Ctrl-C 时 rclpy 的信号处理可能已经关过 context,再关会抛
        # RCLError("rcl_shutdown already called") 把结论表后面糊上一段 traceback。
        if rclpy.ok():
            rclpy.shutdown()
        if proc is not None:
            # 整个进程组一起收,别留孤儿节点占端口/话题
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
