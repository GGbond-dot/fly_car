"""飞车地面分段演示 ③/③ —— 从投货点直行到起飞点 takeoff。

接 step2:飞车此刻停在 drop1、机头 -90°,静止。启动本 launch,沿机头方向直行到起飞点。
  route_test_node   保持 pure-pursuit 算法直行
  servo_set_once    摄像头舵机2 -> 地面 120°

坐标是"本段起点车体系"下的相对位移(重启重新建图,当前位置=新原点、机头=新 +x):
  世界 takeoff(265,-97) - drop1(265,-30) = (0,-67),即沿世界 -y 走 67cm;
  本段起点机头已朝 -90°(=世界 -y),故就是正前方 67cm → 车体系 (67,0),yaw 保持 0(不再转)。
对应任务世界坐标 takeoff=(265,-97) yaw -90。地面段到此结束(下一步才是起飞)。

⚠ 启动前确认飞车没被挪动/转动(机头仍 -90°),否则相对坐标会偏。

用法:  ros2 launch my_launch ground_step3_takeoff.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# [x_cm, y_cm, z_cm, yaw_deg] 本段起点车体系;z=4 地面态
WAYPOINTS = [67.0, 0.0, 4.0, 0.0]
SERVO_SET_SCRIPT = os.path.expanduser("~/kian_flycar/scripts/servo_set_once.py")


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def generate_launch_description():
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py")
    ground_chassis = _include("ground_chassis_pkg", "ground_chassis.launch.py")

    route_test = Node(
        package="activity_control_pkg",
        executable="route_test_node",
        name="route_test_node",
        output="screen",
        parameters=[{
            "waypoints": WAYPOINTS,
            "position_tolerance_cm": 12.0,
            "lookahead_count": 0,
        }],
    )

    camera = ExecuteProcess(   # 摄像头舵机2 -> 地面 120°
        cmd=["python3", SERVO_SET_SCRIPT, "--index", "2", "--angle", "120"],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),   # 飞车域1,与车板(域0)隔离
        fly_carto,
        TimerAction(period=12.0, actions=[ground_chassis, route_test, camera]),
    ])
