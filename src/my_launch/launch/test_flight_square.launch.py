"""飞车【飞】功能测试:原地升到 100cm -> 飞边长 1.5m 正方形 -> 原地下降。

一条命令自包含起飞行链(结构同 demo1):
  fly_carto        雷达 + cartographer  -> TF map->laser_link(水平 xy/yaw 定位)
  uart_to_stm32    与飞控 STM32 通信:发 /height(实测高度), 收 /target_velocity 下发
  position_pid     飞控 PID:吃 /target_position, /height -> 发 /target_velocity
  route_test_node  按 waypoints 参数发 /target_position(飞行航点)

航点 [x_cm, y_cm, z_cm, yaw_deg], yaw 全程 0(平移飞行,机头不变):
  (0,0)升到z=100 -> (150,0,100)->(150,150,100)->(0,150,100)->(0,0,100) -> (0,0)降到z=4
空中航点(z>20)到达判据 = 高度到位 + xy 到位(放宽 yaw);末点 z=4 命令原地下降落地。
pid 的 flight_enable 默认 true, 纯飞行测试不挂地面底盘/仲裁。

用法:  ros2 launch my_launch test_flight_square.launch.py
"""

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


# 升到 100cm -> 边长 1.5m 正方形(z=100) -> 原地降落 [x_cm, y_cm, z_cm, yaw_deg]
FLIGHT_SQUARE = [
    0.0, 0.0, 100.0, 0.0,      # 原地拔高到 100cm
    150.0, 0.0, 100.0, 0.0,    # 边 1
    150.0, 150.0, 100.0, 0.0,  # 边 2
    0.0, 150.0, 100.0, 0.0,    # 边 3
    0.0, 0.0, 100.0, 0.0,      # 边 4 回到起点(仍在空中)
    0.0, 0.0, 4.0, 0.0,        # 原地垂直下降落地
]


def generate_launch_description():
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py")
    uart = _include("uart_to_stm32", "uart_to_stm32.launch.py")
    position_pid = _include("pid_control_pkg", "position_pid_controller.launch.py")

    route_test = Node(
        package="activity_control_pkg",
        executable="route_test_node",
        name="route_test_node",
        output="screen",
        parameters=[{"waypoints": FLIGHT_SQUARE}],
    )

    return LaunchDescription([
        fly_carto,
        # 等雷达/carto 出 TF 再上 uart/飞控/航点(carto 节点在 fly_carto 内 ~10s 启动)
        TimerAction(
            period=12.0,
            actions=[uart, position_pid, route_test],
        ),
    ])
