from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = {
        "serial_port": "/dev/ttyS3",
        "baud_rate": 921600,
        "frame_id": "laser_frame",
    }

    return LaunchDescription([
        Node(
            package="laser_array_pkg",
            executable="laser_array_driver",
            name="laser_array_node",
            output="screen",
            parameters=[params],
        )
    ])
