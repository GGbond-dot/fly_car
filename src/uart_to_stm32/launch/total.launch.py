import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def _locate_launch(package_name: str, filename: str):
    share = Path(get_package_share_directory(package_name))
    launch_path = share / "launch" / filename
    if not launch_path.is_file():
        raise FileNotFoundError(f"{launch_path} 不存在，请检查文件名和路径")
    return str(launch_path)

def generate_launch_description():
    bluesea2_launch = _locate_launch("bluesea2", "udp_lidar.launch")
    carto_launch = _locate_launch("cartographer_ros", "backpack_2d.launch.py")
    uart_launch = _locate_launch("uart_to_stm32", "uart_to_stm32.launch.py")
    pid_ctrl_launch = _locate_launch("pid_control_pkg", "position_pid_controller.launch.py")
    # activity_control_launch = _locate_launch("activity_control_pkg", "route_target_publisher.launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bluesea2_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(carto_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(uart_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pid_ctrl_launch)),
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(activity_control_launch))
    ])
