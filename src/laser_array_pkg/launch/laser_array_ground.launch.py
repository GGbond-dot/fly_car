from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ⚠️ TODO(上板确认): 面阵激光的串口。ttyS3 被底盘 $VW/舵机占用、ttyS6 被
    # STM32 占用,都不能填。上板后 `ls /dev/ttyS* /dev/ttyUSB*` 找到面阵激光那个口,
    # 用 `serial_port:=/dev/ttyXXX` 覆盖,或直接改这里的 default_value。
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyS1",
            description="面阵激光串口(占位 ttyS1,上板务必确认!勿用 ttyS3/ttyS6)"),
        DeclareLaunchArgument(
            "baud_rate", default_value="921600",
            description="面阵激光波特率"),
        Node(
            package="laser_array_pkg",
            executable="laser_array_ground_node",
            name="laser_array_ground_node",
            output="screen",
            parameters=[{
                "serial_port": serial_port,
                "baud_rate": baud_rate,
                # 空间滤波
                "percentile": 1.0,        # 无双峰时: 1.0=最大值, 0.8=80分位(现版恒取max,仅兼容)
                "cluster_gap": 0.20,      # m, 相邻束差距 > 此值认为柱子/地面双峰
                # 时间滤波
                "max_slew_rate": 1.5,     # m/s, 高度变化速率上限
                "ema_alpha": 0.6,         # 0~1, 越大越跟手
                "jump_threshold": 0.15,   # m, 降幅超此值 + 无远簇 -> 冻结输出
                "max_hold_frames": 20,    # 最多保持上一帧多少帧 (50Hz下20帧=0.4s)
                # 障碍检测
                "obstacle_margin": 0.20,  # m
                # 日志
                "log_period_sec": 0.5,    # 高度日志打印间隔
            }],
        )
    ])
