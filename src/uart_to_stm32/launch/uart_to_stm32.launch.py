from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='uart_to_stm32',
            executable='uart_to_stm32_node',
            name='uart_to_stm32',
            parameters=[
                {'update_rate': 100.0},
                {'source_frame': 'map'},
                {'target_frame': 'laser_link'}
            ],
            output='screen'
        )
    ])
