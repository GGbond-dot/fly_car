from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 高度源标志位:false=STM32 单点激光(默认/原样),true=面阵激光(替换 /height + 回传 STM32)。
    # 也可运行时热切: ros2 param set /uart_to_stm32 use_laser_array_height true
    use_laser = LaunchConfiguration('use_laser_array_height')
    return LaunchDescription([
        DeclareLaunchArgument('use_laser_array_height', default_value='false',
                              description='true=用面阵激光高度替换/height并回传STM32'),
        Node(
            package='uart_to_stm32',
            executable='uart_to_stm32_node',
            name='uart_to_stm32',
            parameters=[
                {'update_rate': 100.0},
                {'source_frame': 'map'},
                {'target_frame': 'laser_link'},
                {'use_laser_array_height': use_laser},
            ],
            output='screen'
        )
    ])
