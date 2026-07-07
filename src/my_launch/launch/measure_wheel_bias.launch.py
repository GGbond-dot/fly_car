"""左右轮恒定速度差(转向 bias)开环测量 —— 一条命令起全套。

起 fly_carto(出 TF) + chassis_bridge(/cmd_vel->$VW) + bias_probe(自动跑几段直线量 yaw 漂移)。
⚠ 不起 diff_drive_controller/chassis_mux —— 探针直接发 /cmd_vel 开环驱动,避免闭环干扰。

用法:
  ros2 launch my_launch measure_wheel_bias.launch.py
  ros2 launch my_launch measure_wheel_bias.launch.py speeds:=0.15 drive_s:=6.0   # 只测单速
  ros2 launch my_launch measure_wheel_bias.launch.py speeds:=0.10,0.15,0.20

⚠ 车前方留 2~3m 空地(开环会走弧线,横向可能甩 0.5m)。
跑完看终端 [bias_probe] 打印的"建议 w_bias_rps",填进 diff_drive_controller:
  ros2 param set /diff_drive_controller w_bias_rps <值>   (满意后落进 ground_chassis.launch.py)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    speeds = LaunchConfiguration("speeds")
    drive_s = LaunchConfiguration("drive_s")
    warmup_s = LaunchConfiguration("warmup_s")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyS3"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("speeds", default_value="0.10,0.15,0.20"),
        DeclareLaunchArgument("drive_s", default_value="5.0"),
        DeclareLaunchArgument("warmup_s", default_value="8.0"),

        # 雷达 + carto → TF map<-laser_link
        _include("my_carto_pkg", "fly_carto.launch.py"),

        # 串口桥:订阅 /cmd_vel 转 $VW(与正式跑同款)。不起 controller/mux。
        Node(
            package="ground_chassis_pkg",
            executable="chassis_bridge.py",
            name="chassis_bridge",
            output="screen",
            arguments=[
                "--port", port,
                "--baud", baud,
                "--chassis-timeout-ms", "500",
            ],
        ),

        # 测量探针:自动按 speeds 各跑 drive_s 秒开环直线,算 yaw 漂移率 → 建议 w_bias
        Node(
            package="ground_chassis_pkg",
            executable="bias_probe.py",
            name="bias_probe",
            output="screen",
            parameters=[{
                "speeds": ParameterValue(speeds, value_type=str),
                "drive_s": ParameterValue(drive_s, value_type=float),
                "warmup_s": ParameterValue(warmup_s, value_type=float),
            }],
        ),
    ])
