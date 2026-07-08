"""灾区两次投放 —— 地面段统一入口(飞车侧,当前阶段:不起飞)。

一条命令把地面段全起来,跑到**起飞前那一刻**为止:
  fly_carto        雷达 + cartographer -> TF map->laser_link
  ground_chassis   chassis_bridge($VW + /servo_cmd)+ diff_drive_controller + chassis_mux
  relief_drop_mission_node  航点队列 + 任务状态机(ground_only=true:收到 /resupply_done 即停在起飞点)
  xmachine_bridge  跨机原生 UDP 桥(叫车 /resupply_request、收车 /resupply_done)

不含飞控链(uart_to_stm32 / pid):本阶段飞车不飞,mission 到起飞点等补给完成就结束。
飞行段以后再开(mission 参数 ground_only:=false + 另起 uart/pid,见 relief_drop_mission_design.md)。

用法:  ros2 launch my_launch relief_drop_ground.launch.py
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
    ground_chassis = _include("ground_chassis_pkg", "ground_chassis.launch.py")
    mission = _include("activity_control_pkg", "relief_drop_mission.launch.py")

    return LaunchDescription([
        fly_carto,
        # 等雷达/carto 出 TF(carto 节点在 fly_carto 内 ~10s 起)再上底盘 + 任务
        TimerAction(
            period=12.0,
            actions=[ground_chassis, mission],
        ),
    ])
