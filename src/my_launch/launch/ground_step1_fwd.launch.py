"""飞车地面分段演示 ①/③ —— 从起点前进到货处 fwd(开环直行版)。

直线用开环:只起 chassis_bridge,open_loop_drive 定频发 /cmd_vel 走固定距离,不用
carto/pure-pursuit —— 直线开环比闭环算法准(转弯才需要算法,见 step2/step3)。
启动即把摄像头舵机2 摆到 120°(地面略朝下)。

飞车在起点(静止、机头朝前)启动即前进 DISTANCE_CM;对应任务世界坐标 fwd=(245,0)。
速度/距离标不准就现场调下面 SPEED_MPS / DISTANCE_CM。

⚠ 分段接续:跑完飞车停在货处、机头 0°,别手动挪动/转动,直接跑 step2。

用法:  ros2 launch my_launch ground_step1_fwd.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

SPEED_MPS = "0.16"
DISTANCE_CM = "245"
DRIVE_SCRIPT = os.path.expanduser("~/kian_flycar/scripts/open_loop_drive.py")
SERVO_SET_SCRIPT = os.path.expanduser("~/kian_flycar/scripts/servo_set_once.py")


def generate_launch_description():
    chassis_bridge = Node(
        package="ground_chassis_pkg",
        executable="chassis_bridge.py",
        name="chassis_bridge",
        output="screen",
        arguments=["--port", "/dev/ttyS3", "--baud", "115200", "--chassis-timeout-ms", "500"],
    )

    drive = ExecuteProcess(
        cmd=["python3", DRIVE_SCRIPT, "--distance-cm", DISTANCE_CM, "--speed", SPEED_MPS],
        output="screen",
    )

    camera = ExecuteProcess(   # 摄像头舵机2 -> 地面 120°
        cmd=["python3", SERVO_SET_SCRIPT, "--index", "2", "--angle", "120"],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),   # 飞车域1,与车板(域0)隔离
        chassis_bridge,
        # 等串口开好($MODE,VW 就绪)再发速度与舵机,避免前几帧丢失
        TimerAction(period=3.0, actions=[camera, drive]),
    ])
