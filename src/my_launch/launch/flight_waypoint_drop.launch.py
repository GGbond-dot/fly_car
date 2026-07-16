"""飞车【航点自动飞 + 中途投货】—— 架构同 test_flight_square(fly_carto+uart+pid 飞控闭环),
只把发航点的 route_test_node 换成 flight_waypoint_drop.py:持续发当前航点、到位推进,
到投货点悬停开货舱投货、投完再飞走,末点垂直下降。

  fly_carto              雷达+carto -> TF + /velocity_map
  uart_to_stm32          飞控通信:下发当前速度/目标速度,发 /height
  position_pid           吃 /target_position + /height -> 发 /target_velocity(飞控 PID)
  flight_waypoint_drop   本任务航点源(见脚本顶部 WAYPOINTS / DROP_INDEX)
  chassis_bridge         转发 /servo_cmd → $SERVO(投货/摄像头舵机走 ttyS3 地面板)
  servo_set_once         起飞前摄像头舵机2 -> 180°(完全朝下)

⚠ 一起 launch 飞车就会按航点自动起飞升空(首点 0,0,100)——跟 test_flight_square 一样。
  确认场地/桨安全、人在遥控器旁可随时接管再跑。
⚠ 投货/摄像头舵机走 ttyS3 地面板,地面板需通电。
⚠ 航点/投货点/降落改 scripts/flight_waypoint_drop.py 顶部(改完 syncpi 即可,不用 build)。

用法:  ros2 launch my_launch flight_waypoint_drop.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

FLIGHT_SCRIPT = os.path.expanduser("~/kian_flycar/scripts/flight_waypoint_drop.py")
SERVO_SET_SCRIPT = os.path.expanduser("~/kian_flycar/scripts/servo_set_once.py")


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def generate_launch_description():
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py")
    uart = _include("uart_to_stm32", "uart_to_stm32.launch.py")
    position_pid = _include("pid_control_pkg", "position_pid_controller.launch.py")

    chassis_bridge = Node(
        package="ground_chassis_pkg",
        executable="chassis_bridge.py",
        name="chassis_bridge",
        output="screen",
        arguments=["--port", "/dev/ttyS3", "--baud", "115200", "--chassis-timeout-ms", "500"],
    )

    flight = ExecuteProcess(
        cmd=["python3", FLIGHT_SCRIPT],
        output="screen",
    )

    camera = ExecuteProcess(   # 摄像头舵机2 -> 飞行 180°(完全朝下)
        cmd=["python3", SERVO_SET_SCRIPT, "--index", "2", "--angle", "180"],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),   # 飞车域1,与车板(域0)隔离
        fly_carto,
        # 等雷达/carto 出 TF、/velocity_map 再上飞控链、航点、舵机
        TimerAction(period=12.0, actions=[uart, position_pid, chassis_bridge, camera, flight]),
    ])
