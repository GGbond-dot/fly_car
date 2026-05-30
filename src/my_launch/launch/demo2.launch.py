import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def _include_launch(package_name: str, filename: str) -> IncludeLaunchDescription:
    package_share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, "launch", filename))
    )


def generate_launch_description():
    return LaunchDescription([
        _include_launch("my_carto_pkg", "fly_carto.launch.py"),
        _include_launch("uart_to_stm32", "uart_to_stm32.launch.py"),
    ])
