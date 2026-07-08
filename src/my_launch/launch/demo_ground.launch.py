"""完整地面演示 —— 飞车板一键入口(演示当天飞车侧只跑这一条)。

内部 include(不复制节点,原模块化 launch 仍可单独跑调试):
  relief_drop_ground.launch.py   fly_carto + ground_chassis + mission_node + xmachine_bridge
  yolo_detector.launch.py        难民识别 -> /yolo_detector/detections(视觉握手必需)

跑到起飞前那一刻为止(ground_only,不起飞)。配合车板 car_launch/demo_ground.launch.py。

前置:连路由器、与车板互 ping(飞车 .171 / 车 .161)、全域 0、RMW fastrtps。

用法:  ros2 launch my_launch demo_ground.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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
        ground,
        yolo,
    ])
