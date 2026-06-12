"""启动 Cartographer 与折线障碍检测，用于固定场地图上的检测测试。"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def _include_launch(package_name, filename, launch_arguments=None):
    package_share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, "launch", filename)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        _include_launch("my_carto_pkg", "fly_carto.launch.py"),
        _include_launch(
            "obstacle_detector_pkg",
            "obstacle_detector.launch.py",
            {
                # 固定检测起点为 map (0, 0)，整张地图范围。
                "roi_x_min_m": "0.0",
                "roi_x_max_m": "5.0",
                "roi_y_min_m": "-4.0",
                "roi_y_max_m": "0.0",
            },
        ),
    ])
