"""完整地面演示 —— 飞车板一键入口(演示当天飞车侧只跑这一条)。

内部 include(不复制节点,原模块化 launch 仍可单独跑调试):
  relief_drop_ground.launch.py   fly_carto + ground_chassis + mission_node + xmachine_bridge
  yolo_detector.launch.py        难民识别 -> /yolo_detector/detections(视觉握手必需)

跑到起飞前那一刻为止(ground_only,不起飞)。配合车板 car_launch/demo_ground.launch.py。

前置:连路由器、与车板互 ping(飞车 .171 / 车 .161)、RMW fastrtps。
域隔离:飞车整条链路走 ROS_DOMAIN_ID=1(本文件顶部注入),车板保持默认域 0 —— 两机
  DDS 互不可见,飞车的 /target_position 等本地话题不会漏到车板把补给车带走;跨机握手
  (/mission_start、/resupply_request、/resupply_done)由 xmachine_bridge 走原生 UDP,与域无关。
  ⚠ 在飞车板手动跑 ros2 命令(topic echo / node list)要先 `export ROS_DOMAIN_ID=1` 才看得到本机节点。

用法:  ros2 launch my_launch demo_ground.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def generate_launch_description():
    ground = _include("my_launch", "relief_drop_ground.launch.py")
    yolo = _include("yolo_detector_pkg", "yolo_detector.launch.py")

    return LaunchDescription([
        # 飞车整条链路进域 1(车板留在默认域 0),DDS 与车板隔离;须在起任何节点之前设置。
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
        ground,
        yolo,
    ])
