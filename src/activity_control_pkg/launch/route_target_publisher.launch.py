from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # pragma: no cover
    LaunchDescription = Any
    Node = Any


def generate_launch_description():
    launch_module = import_module("launch")
    launch_ros_actions = import_module("launch_ros.actions")
    LaunchDescription = getattr(launch_module, "LaunchDescription")
    Node = getattr(launch_ros_actions, "Node")

    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="route_target_publisher_node",
            name="route_target_publisher",
            output="screen",
            parameters=[
                {
                    "map_frame": "map",
                    "laser_link_frame": "laser_link",
                    "output_topic": "/target_position",
                    "position_tolerance_cm": 6.0,
                    "yaw_tolerance_deg": 5.0,
                    "height_tolerance_cm": 6.0,
                }
            ],
        )
    ])
