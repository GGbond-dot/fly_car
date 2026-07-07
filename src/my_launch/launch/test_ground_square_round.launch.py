"""飞车【跑】功能测试(圆角/不原地转版):地面差速底盘走边长 1m、四角倒圆的方形。

与 test_ground_square.launch.py 同栈(fly_carto + ground_chassis + route_test),
区别只在航点:把四个直角换成 1/4 圆弧,每段航点间转向 <45°(align_gate),让底盘
一边走一边转(走弧线),全程不进入 v=0 原地转分支。配合 round_corner_gate.yaml 把
route_target_publisher 的 yaw 门控放宽,车按 xy 到点即推进,弧线连续通过。

代价:四角是圆的(半径 R),不是直角,轨迹比 1m 方形略大。想要标准直角(接受原地转)
用 test_ground_square.launch.py。

航点由 rounded_square() 生成:边长 L=100cm、圆角半径 R=25cm、弧每 30° 采一点,
yaw = 切线(行进)方向, z=0。

用法:  ros2 launch my_launch test_ground_square_round.launch.py
"""

import math
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def rounded_square(side_cm=100.0, radius_cm=25.0, z_cm=0.0, arc_step_deg=30.0):
    """生成边长 side、四角倒圆(半径 radius)的方形轮廓航点(CCW,起于 (radius,0))。

    返回扁平数组 [x_cm, y_cm, z_cm, yaw_deg, ...]。yaw 取相邻点连线方向(行进切线)。
    """
    L, R = side_cm, radius_cm
    pts = []

    def add(x, y):
        pts.append((x, y))

    def arc(cx, cy, a0_deg, a1_deg):
        # 从 a0 到 a1 每 arc_step 采点(含端点),角度按弧度圆上 (cx,cy)+R*(cos,sin)
        n = max(1, int(round(abs(a1_deg - a0_deg) / arc_step_deg)))
        for i in range(n + 1):
            a = math.radians(a0_deg + (a1_deg - a0_deg) * i / n)
            add(cx + R * math.cos(a), cy + R * math.sin(a))

    # 下边 (+x) -> 右下弧 -> 右边 (+y) -> 右上弧 -> 上边 (-x) -> 左上弧 -> 左边 (-y) -> 左下弧
    add(R, 0.0)
    add(L - R, 0.0)
    arc(L - R, R, -90.0, 0.0)      # 右下角: 朝 +x 转到 +y
    add(L, L - R)
    arc(L - R, L - R, 0.0, 90.0)   # 右上角: +y 转到 -x
    add(R, L)
    arc(R, L - R, 90.0, 180.0)     # 左上角: -x 转到 -y
    add(0.0, R)
    arc(R, R, 180.0, 270.0)        # 左下角: -y 转回 +x
    add(R, 0.0)                    # 闭合回起点

    # 去重相邻重复点,再算每点 yaw = 指向下一点方向;末点沿用前一段方向
    dedup = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 1e-3:
            dedup.append(p)

    flat = []
    for i, (x, y) in enumerate(dedup):
        nx, ny = dedup[min(i + 1, len(dedup) - 1)]
        if i == len(dedup) - 1:
            px, py = dedup[i - 1]
            yaw = math.degrees(math.atan2(y - py, x - px))
        else:
            yaw = math.degrees(math.atan2(ny - y, nx - x))
        flat += [round(x, 1), round(y, 1), z_cm, round(yaw, 1)]
    return flat


GROUND_SQUARE_ROUND = rounded_square()


def generate_launch_description():
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py")
    ground_chassis = _include("ground_chassis_pkg", "ground_chassis.launch.py")

    gate_yaml = os.path.join(
        FindPackageShare(package="my_launch").find("my_launch"),
        "config", "round_corner_gate.yaml",
    )

    route_test = Node(
        package="activity_control_pkg",
        executable="route_test_node",
        name="route_test_node",
        output="screen",
        parameters=[{
            "waypoints": GROUND_SQUARE_ROUND,
            # 与普通车一致:提前推进航点,兼容前驱轴心转向带来的雷达点偏移。
            "position_tolerance_cm": 15.0,
            # 弧线连续通过:按 xy 到点即推进,yaw 不参与门控(否则密航点处 yaw 进不了 5°
            # → 赖在近点不走 → 方位角 atan2 超敏疯摆)。设大值=实质关掉 yaw 门。
            "yaw_tolerance_deg": 180.0,
            # pure-pursuit:在 /target_position 后追加接下来 3 个航点 xy,供控制器取前视点、自动圆角。
            "lookahead_count": 3,
        }, gate_yaml],
    )

    return LaunchDescription([
        fly_carto,
        TimerAction(
            period=12.0,
            actions=[ground_chassis, route_test],
        ),
    ])
