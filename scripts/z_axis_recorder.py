#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
z 轴数据记录器 —— 掉高度分析

把飞行中 z 轴所有关键量摊开、实时打印、并存 CSV 供事后画图分析。
(只有这架飞机掉高度、其它都正常 → 多半是本机硬件。这脚本给你数据,
 换硬件前后各录一次可直接对比。)

采集/记录的 z 轴关键量:
  target_z   目标高度(cm)   来自 /target_position[2]
  height     当前高度(cm)   来自 /height (STM32 单点激光, 飞控实际吃的)
  err_z      = target_z - height  (飞控看到的高度误差)
  vz_cmd     飞控下发的 z 速度指令(cm/s)  来自 /target_velocity[2]
             ↑ >0=命令上升  <0=命令下降 —— 判"动力不足"还是"数据虚高"的关键
  laser      面阵激光高度(cm)  来自 /laser_array/ground_height(没接线则空)
  dH/dt      当前高度的实际变化率(cm/s, 负=真在下降)

用法(飞车板上,飞控 PID + uart_to_stm32 已在跑):
  python3 z_axis_recorder.py                 # 实时打印 + Ctrl-C 结束存 CSV
  python3 z_axis_recorder.py --hz 20         # 20Hz 记录(默认10)
  python3 z_axis_recorder.py --csv /tmp/a.csv
"""
import argparse
import csv
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray


class ZAxisRecorder(Node):
    def __init__(self, rate_hz, csv_path):
        super().__init__("z_axis_recorder")
        # 最近值缓存
        self.target_z = None
        self.height = None
        self.stm32_raw = None
        self.vz_cmd = None
        self.laser = None
        # 历史 (t, height) 用于算 dH/dt;全量 rows 存 CSV
        self.height_hist = []
        self.rows = []
        self.t0 = time.monotonic()
        self.csv_path = csv_path
        self._last_print = 0.0

        self.create_subscription(Int16, "/height", self._on_height, 10)
        self.create_subscription(Int16, "/height_raw_stm32", self._on_stm32_raw, 10)
        self.create_subscription(Int16, "/laser_array/ground_height", self._on_laser, 10)
        self.create_subscription(Float32MultiArray, "/target_position", self._on_tgt, 10)
        self.create_subscription(Float32MultiArray, "/target_velocity", self._on_vel, 10)

        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._tick)
        print(f"\n{'t(s)':>6} {'tgt_z':>6} {'height':>7} {'err_z':>6} "
              f"{'vz_cmd':>7} {'laser':>6} {'dH/dt':>7}")
        print("-" * 54)

    def _on_height(self, m):
        self.height = int(m.data)
        self.height_hist.append((time.monotonic(), self.height))

    def _on_stm32_raw(self, m):
        self.stm32_raw = int(m.data)

    def _on_laser(self, m):
        self.laser = int(m.data)

    def _on_tgt(self, m):
        if len(m.data) >= 3:
            self.target_z = float(m.data[2])

    def _on_vel(self, m):
        if len(m.data) >= 3:
            self.vz_cmd = float(m.data[2])

    def _dh_dt(self):
        # 用最近 ~0.4s 的高度历史做斜率
        now = time.monotonic()
        window = [(t, h) for t, h in self.height_hist if now - t <= 0.4]
        if len(window) < 2:
            return None
        dt = window[-1][0] - window[0][0]
        if dt <= 0:
            return None
        return (window[-1][1] - window[0][1]) / dt

    def _tick(self):
        t = time.monotonic() - self.t0
        err = (self.target_z - self.height) if (self.target_z is not None and self.height is not None) else None
        dh = self._dh_dt()
        self.rows.append(dict(t=t, target_z=self.target_z, height=self.height,
                              stm32_raw=self.stm32_raw, err_z=err, vz_cmd=self.vz_cmd,
                              laser=self.laser, dh_dt=dh))

        # 实时打印(限 ~2Hz 不刷屏)
        if t - self._last_print >= 0.5:
            self._last_print = t

            def f(x, w, p=0):
                return (f"{x:>{w}.{p}f}" if isinstance(x, (int, float)) else f"{'—':>{w}}")
            print(f"{t:>6.1f} {f(self.target_z,6)} {f(self.height,7)} {f(self.err_z_val(err),6)} "
                  f"{f(self.vz_cmd,7,0)} {f(self.laser,6)} {f(dh,7,0)}")

    @staticmethod
    def err_z_val(err):
        return err


def summarize(node):
    rows = node.rows
    line = "─" * 60
    print("\n" + line)
    print("  z 轴数据小结")
    print(line)

    def col(key):
        return [r[key] for r in rows if r[key] is not None]

    for key, label in [("target_z", "目标高度"), ("height", "当前高度(/height)"),
                       ("stm32_raw", "STM32单点原值"), ("laser", "面阵高度"),
                       ("vz_cmd", "vz指令"), ("dh_dt", "实际dH/dt")]:
        v = col(key)
        if v:
            print(f"  {label:<18} 均值{statistics.mean(v):>7.1f}  min{min(v):>7.1f}  max{max(v):>7.1f}  ({len(v)}样本)")
        else:
            print(f"  {label:<18} 无数据")

    # 抓最大下降段(基于 /height)
    hh = [(r["t"], r["height"]) for r in rows if r["height"] is not None]
    drop = _biggest_drop(hh)
    if drop:
        t0, t1, top, bot, dcm = drop
        dur = t1 - t0
        seg_vz = [r["vz_cmd"] for r in rows if t0 - 0.3 <= r["t"] <= t1 + 0.3 and r["vz_cmd"] is not None]
        print(f"\n  最大下降: {top:.0f}→{bot:.0f}cm 跌{dcm:.0f}cm 用时{dur:.1f}s (≈{dcm/dur if dur>0 else 0:.0f}cm/s)")
        if seg_vz:
            mv = statistics.mean(seg_vz)
            print(f"  该段飞控 vz 均值 {mv:+.0f}cm/s  →  ", end="")
            if mv > 5:
                print("命令上升却仍掉 = 偏【动力/电流】(换硬件方向对)")
            elif mv < -5:
                print("飞控在命令下降 = 偏【高度数据虚高】,核对 /height 与真实高度")
            else:
                print("方向不明显,多录一次")
    print(line)

    # 存 CSV
    if node.csv_path and rows:
        with open(node.csv_path, "w", newline="") as fp:
            w = csv.writer(fp)
            w.writerow(["t_s", "target_z", "height", "stm32_raw", "err_z", "vz_cmd", "laser", "dH_dt"])
            for r in rows:
                w.writerow([f"{r['t']:.3f}",
                            _s(r["target_z"]), _s(r["height"]), _s(r["stm32_raw"]),
                            _s(r["err_z"]), _s(r["vz_cmd"]), _s(r["laser"]), _s(r["dh_dt"])])
        print(f"  CSV 已存: {node.csv_path}  ({len(rows)}行) —— 可直接导入 Excel/画图")
    print(line + "\n")


def _s(x):
    return "" if x is None else (f"{x:.2f}" if isinstance(x, float) else x)


def _biggest_drop(hh, min_drop_cm=40.0, max_dur_s=6.0):
    if len(hh) < 3:
        return None
    best = None
    for i in range(len(hh)):
        ti, vi = hh[i]
        for j in range(i + 1, len(hh)):
            tj, vj = hh[j]
            if tj - ti > max_dur_s:
                break
            d = vi - vj
            if d >= min_drop_cm and (best is None or d > best[4]):
                best = (ti, tj, vi, vj, d)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=10.0, help="记录频率(默认10Hz)")
    ap.add_argument("--csv", default=f"z_axis_log_{int(time.time())}.csv",
                    help="CSV 输出路径(默认当前目录带时间戳)")
    args = ap.parse_args()

    rclpy.init()
    node = ZAxisRecorder(args.hz, args.csv)
    node.get_logger().info("记录中... 飞一段复现掉高度,Ctrl-C 结束")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        summarize(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
