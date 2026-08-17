"""独立运行固定搜救路线，不依赖 AI Agent、地面车或跨机通信。

用法:
  ros2 launch my_launch fixed_rescue_route.launch.py
  ros2 launch my_launch fixed_rescue_route.launch.py start_delay_s:=12.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_launch.fixed_rescue_route import (
    DEFAULT_START_DELAY_S,
    flattened_waypoints,
)


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def generate_launch_description():
    start_delay = LaunchConfiguration("start_delay_s")
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py")
    ground_chassis = _include("ground_chassis_pkg", "ground_chassis.launch.py")
    uart = _include("uart_to_stm32", "uart_to_stm32.launch.py")
    position_pid = _include(
        "pid_control_pkg", "position_pid_controller.launch.py"
    )
    route_executor = Node(
        package="activity_control_pkg",
        executable="route_test_node",
        name="fixed_rescue_route_executor",
        output="screen",
        parameters=[{
            "preload_waypoints": True,
            "waypoints": flattened_waypoints(),
            "position_tolerance_cm": 12.0,
            "lookahead_count": 0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_delay_s",
            default_value=str(DEFAULT_START_DELAY_S),
            description="启动运动链前等待 Cartographer/TF 的秒数",
        ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
        LogInfo(msg=[
            "固定搜救路线测试将在 ",
            start_delay,
            " 秒后启动；实际起飞仍由遥控器开关控制。",
        ]),
        fly_carto,
        TimerAction(
            period=start_delay,
            actions=[ground_chassis, uart, position_pid, route_executor],
        ),
    ])
