"""飞车【遥控手飞】稳定性测试:结构同 test_flight_square,只去掉航点节点(route_test_node)。

  fly_carto        雷达 + cartographer -> TF map->laser_link,并发 /velocity_map(当前速度)
  uart_to_stm32    与飞控 STM32 通信:发 /height(实测高度), 收 /target_velocity 下发
  position_pid     飞控 PID:吃 /target_position, /height -> 发 /target_velocity

去掉了 route_test_node,所以【没人发 /target_position】,pid 不会输出自动航点指令,
飞控只拿到当前速度反馈,航向/油门全交给遥控器 —— 用来手飞验证飞得稳不稳。
稳了再上 test_flight_square 测自动方形。

用法:  ros2 launch my_launch test_flight_manual.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def generate_launch_description():
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py")
    uart = _include("uart_to_stm32", "uart_to_stm32.launch.py")
    position_pid = _include("pid_control_pkg", "position_pid_controller.launch.py")

    return LaunchDescription([
        fly_carto,
        # 等雷达/carto 出 TF 再上 uart/飞控(carto 节点在 fly_carto 内 ~10s 启动)
        TimerAction(
            period=12.0,
            actions=[uart, position_pid],
        ),
    ])
