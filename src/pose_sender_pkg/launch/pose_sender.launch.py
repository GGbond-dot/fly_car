"""飞车位姿 UDP 发送(供 car 跟随)。target_ip 必须按组网实际配置。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_ip', default_value='192.168.10.161',
                              description='car 在路由器网段的固定地址(飞车自身 192.168.10.171)'),
        DeclareLaunchArgument('target_port', default_value='8888'),
        Node(
            package='pose_sender_pkg',
            executable='pose_sender',
            parameters=[{
                'target_ip': LaunchConfiguration('target_ip'),
                'target_port': LaunchConfiguration('target_port'),
                'rate_hz': 20.0,
                'map_frame': 'map',
                'body_frame': 'laser_link',
            }],
            output='screen',
        ),
    ])
