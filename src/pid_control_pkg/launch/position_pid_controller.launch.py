from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="pid_control_pkg",
            executable="position_pid_controller",
            name="position_pid_controller",
            output="screen",
            parameters=[
                {
                    "control_frequency": 50.0,
                    "map_frame": "map",
                    "laser_link_frame": "laser_link",
                    "position_tolerance": 6.0,
                    "yaw_tolerance": 5.0,
                    "height_tolerance": 6.0,
                    "kp_xy": 0.5,
                    "ki_xy": 0.0,
                    "kd_xy": 0.1,
                    "kp_yaw": 1.0,
                    "ki_yaw": 0.0,
                    "kd_yaw": 0.2,
                    "kp_z": 0.9,
                    "ki_z": 0.0,
                    "kd_z": 0.15,
                    "max_linear_velocity": 33.0,
                    "max_angular_velocity": 30.0,
                    "max_vertical_velocity": 30.0,
                    "max_slow_velocity": 20.0,
                }
            ],
        )
    ])
