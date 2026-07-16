"""飞车地面分段演示 ②/③ —— 从货处右转走到投货点 drop1,到位后开货舱丢货。

接 step1:飞车此刻停在货处、机头 0°,静止。启动本 launch:
  route_test_node   保持 pure-pursuit 算法走到 drop1(转弯用算法才准)
  drop_on_arrival   TF 判到 drop1 后发 /servo_cmd 开货舱→停 1.5s→复位(投货1)
  servo_set_once    摄像头舵机2 -> 地面 120°

坐标是"本段起点车体系"下的相对位移(重启重新建图,当前位置=新原点、机头=新 +x):
  世界 drop1(265,-30) - fwd(245,0) = (+20,-30);机头 0° 时车体系=世界系 → (20,-30)。
  目标 yaw=-90(右转对准飞车尾部货箱)。到点后机头 -90°,投货舱触发。
对应任务世界坐标 drop1=(265,-30) yaw -90;投货舵机 1:开180/复位90。

⚠ 启动前确认飞车没被挪动/转动(机头仍 0°),否则相对坐标会偏。
⚠ 跑完停在 drop1、机头 -90°,别动,直接跑 step3。

用法:  ros2 launch my_launch ground_step2_drop1.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# [x_cm, y_cm, z_cm, yaw_deg] 本段起点车体系;z=4 地面态
WAYPOINTS = [20.0, -30.0, 4.0, -90.0]
DROP_SCRIPT = os.path.expanduser("~/kian_flycar/scripts/drop_on_arrival.py")
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

    # 到 drop1(车体系 20,-30)后开货舱投货,与 WAYPOINTS 终点 xy 对齐
    drop = ExecuteProcess(
        cmd=["python3", DROP_SCRIPT,
             "--target-x-cm", "20", "--target-y-cm", "-30", "--tol-cm", "12",
             "--index", "1", "--open-deg", "180", "--close-deg", "90", "--t-drop-s", "1.5"],
        output="screen",
    )

    camera = ExecuteProcess(   # 摄像头舵机2 -> 地面 120°
        cmd=["python3", SERVO_SET_SCRIPT, "--index", "2", "--angle", "120"],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),   # 飞车域1,与车板(域0)隔离
        fly_carto,
        # 等雷达/carto 出 TF 再上底盘链、航点、投货、摄像头舵机
        TimerAction(period=12.0, actions=[ground_chassis, route_test, drop, camera]),
    ])
