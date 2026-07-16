"""飞车固定 L 直线调参 —— **最小** launch,只起磨直线用得上的三样。

只起:
  fly_carto              雷达 + cartographer → TF map→laser_link(算 yaw/位置)
  ground_chassis         chassis_bridge(/dev/ttyS3 $VW 底盘) + diff_drive_controller + chassis_mux
  route_test_node        收 /wildlife/waypoints(l_path_tuning.py 发那条固定 L)→ /target_position

**为什么不用 patrol_ground.launch.py**:那个是跑完整任务的,还起 uart/position_pid(飞控)、
servo_camera_by_mode.py、rescue_drop_sequencer、xmachine_bridge、yolo。这些跟磨直线毫无关系,
而且 servo_camera **也开 /dev/ttyS3**(舵机 $SERVO)—— 跟 chassis_bridge 的 $VW 抢同一个串口,
一崩就把整个 launch 带下水(2026-07-16 上板卡在这)。调参只留这三样,干净。

配对脚本:scripts/l_path_test.sh(起本 launch → 等节点齐 → 跑 l_path_tuning.py)。
⚠⚠ 调参车会真跑,**拆掉全部桨**。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def generate_launch_description():
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py")
    ground_chassis = _include("ground_chassis_pkg", "ground_chassis.launch.py")

    # 航点执行器。preload_waypoints:=false → 不预装演示航点,静等 l_path_tuning.py 下发
    # 那条固定 L 到 /wildlife/waypoints。lookahead_count:=0 → 本路线不走前视(codex 的固定 L
    # 简化逻辑就是锁航向直行 + 原地转,不用 pure-pursuit 前视)。
    route_executor = Node(
        package="activity_control_pkg",
        executable="route_test_node",
        name="route_test_node",
        output="screen",
        parameters=[{
            "preload_waypoints": False,
            "route_topic": "/wildlife/waypoints",
            "position_tolerance_cm": 12.0,
            "lookahead_count": 0,
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
        fly_carto,
        # 等 carto 出 TF(~10s 起)再上底盘和航点执行器 —— 它们要查 TF 才能算,
        # 早起只是空转 warn(不会崩),留 12s 跟 patrol_ground 一致。
        TimerAction(period=12.0, actions=[ground_chassis, route_executor]),
    ])
