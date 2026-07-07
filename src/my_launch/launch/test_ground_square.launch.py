"""飞车【跑】功能测试(真方形/原地转版):地面差速底盘走边长 1m 的正方形。

一条命令自包含起全栈:
  fly_carto        雷达 + cartographer  -> TF map->laser_link(定位)
  ground_chassis   chassis_bridge + diff_drive_controller + chassis_mux
  route_test_node  按 waypoints 参数发 /target_position(方形航点)

航点(map 系, cm, z=0 全程地面), yaw = 到点后要走的【下一条边】方向 —— 差速控制器
"到点即原地转到该 yaw,再直行去下一点",即"先转向再走",每个直角一次 ~90° 原地转:
  起点(0,0)朝0° -> (100,0)转朝+y -> (100,100)转朝-x -> (0,100)转朝-y -> (0,0)停
前驱做不到完美零移动原地转;控制点已设在前轴中点(=旋转中心)使原地转不漂 xy。
不想原地转、可接受圆角,见 test_ground_square_round.launch.py。

chassis_mux 见目标 z=0 且无 /height 上报 -> 判地面态, ground_enable=true 放行底盘。
纯地面测试不需要 uart/飞控链。

用法:  ros2 launch my_launch test_ground_square.launch.py
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


# 边长 1m 正方形 [x_cm, y_cm, z_cm, yaw_deg]; yaw = 到点后要走的下一条边方向
GROUND_SQUARE = [
    100.0, 0.0, 0.0, 90.0,     # 到(100,0)后转朝 +y
    100.0, 100.0, 0.0, 180.0,  # 到(100,100)后转朝 -x
    0.0, 100.0, 0.0, -90.0,    # 到(0,100)后转朝 -y
    0.0, 0.0, 0.0, -90.0,      # 回起点即停(到达朝向即 -y,不再多转)
]


def generate_launch_description():
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py")
    ground_chassis = _include("ground_chassis_pkg", "ground_chassis.launch.py")

    route_test = Node(
        package="activity_control_pkg",
        executable="route_test_node",
        name="route_test_node",
        output="screen",
        parameters=[{
            "waypoints": GROUND_SQUARE,
            # 与普通车一致:容忍前驱轴心转向时雷达点绕轴心产生的小位移。
            "position_tolerance_cm": 15.0,
            # pure-pursuit:追加接下来 3 个航点 xy,控制器取前视点 → 硬直角被自动圆角化、连续通过。
            "lookahead_count": 3,
        }],
    )

    return LaunchDescription([
        fly_carto,
        # 等雷达/carto 起来出 TF 再上底盘链与航点(carto 节点在 fly_carto 内 ~10s 启动)
        TimerAction(
            period=12.0,
            actions=[ground_chassis, route_test],
        ),
    ])
