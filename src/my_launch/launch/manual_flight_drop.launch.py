"""飞车【手飞】—— 雷达速度反馈给飞控自稳,遥控器手控方向油门,话题控制第二次投货。

不用 pid、不算航点。只起产生"当前速度"最关键的两条:
  fly_carto        雷达 + cartographer -> TF map->laser_link,并发 /velocity_map(当前速度)
  uart_to_stm32    把 /velocity_map(当前速度)+TF 下发飞控做速度反馈自稳;发 /height
去掉 position_pid,所以没有 /target_velocity 目标指令 —— 飞控只拿当前速度反馈,
方向/油门全交给遥控器(uart 下发只看 velocity_valid+yaw_valid,不依赖 /target_velocity,已确认安全)。

投货与摄像头(舵机走 ttyS3 地面板,用 chassis_bridge 转发 /servo_cmd → $SERVO):
  servo_set_once      起飞前把摄像头舵机2 摆到 180°(完全朝下)
  servo_drop_on_topic 手飞到难民上方,发 /drop_now 即投第二次货(舵机1 开180→停1.5s→复位90)

第二次投货触发(飞车在域1,先 export):
  export ROS_DOMAIN_ID=1
  ros2 topic pub --once /drop_now std_msgs/msg/Bool "{data: true}"

⚠ 地面板(ttyS3)需通电,否则 chassis_bridge 开不了串口、投不了货。飞机在空中,地面轮子停/空转无碍。

用法:  ros2 launch my_launch manual_flight_drop.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SERVO_SET_SCRIPT = os.path.expanduser("~/kian_flycar/scripts/servo_set_once.py")
DROP_SCRIPT = os.path.expanduser("~/kian_flycar/scripts/servo_drop_on_topic.py")


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def generate_launch_description():
    fly_carto = _include("my_carto_pkg", "fly_carto.launch.py")
    uart = _include("uart_to_stm32", "uart_to_stm32.launch.py")

    # 舵机走 ttyS3 地面板;chassis_bridge 转发 /servo_cmd → $SERVO(投货/摄像头都靠它)
    chassis_bridge = Node(
        package="ground_chassis_pkg",
        executable="chassis_bridge.py",
        name="chassis_bridge",
        output="screen",
        arguments=["--port", "/dev/ttyS3", "--baud", "115200", "--chassis-timeout-ms", "500"],
    )

    camera = ExecuteProcess(   # 摄像头舵机2 -> 飞行 180°(完全朝下)
        cmd=["python3", SERVO_SET_SCRIPT, "--index", "2", "--angle", "180"],
        output="screen",
    )

    drop = ExecuteProcess(     # 话题触发第二次投货(舵机1)
        cmd=["python3", DROP_SCRIPT,
             "--index", "1", "--open-deg", "180", "--close-deg", "90", "--t-drop-s", "1.5"],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),   # 飞车域1,与车板(域0)隔离
        fly_carto,
        # 等雷达/carto 出 TF、/velocity_map 再上飞控串口与舵机链
        TimerAction(period=12.0, actions=[uart, chassis_bridge, camera, drop]),
    ])
