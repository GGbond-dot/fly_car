import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def _include_launch(package_name: str, filename: str) -> IncludeLaunchDescription:
    package_share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, "launch", filename))
    )


def generate_launch_description():
    fly_carto_launch = _include_launch("my_carto_pkg", "fly_carto.launch.py")
    uart_to_stm32_launch = _include_launch("uart_to_stm32", "uart_to_stm32.launch.py")
    position_pid_launch = _include_launch("pid_control_pkg", "position_pid_controller.launch.py")
    route_test_launch = _include_launch("activity_control_pkg", "route_test.launch.py")

    return LaunchDescription([
        fly_carto_launch,
        TimerAction(
            period=3.0,
            actions=[
                uart_to_stm32_launch,
                position_pid_launch,
                route_test_launch,
            ],
        ),
    ])
