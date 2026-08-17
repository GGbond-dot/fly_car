"""四驱底盘"走不走得直"测试 —— 一条命令跑完,脚本自己打结论表。

  ros2 launch ground_chassis_pkg test_straight_line.launch.py
  ros2 launch ground_chassis_pkg test_straight_line.launch.py distance_cm:=300

只起三个节点:串口桥 + 四轮监控 + 测试脚本。
**不起 diff_drive_controller / chassis_mux** —— 测的是底盘开环直不直,
闭环在场会把机械的歪自动纠回来,测出来就不是底盘本身了(见脚本头注释)。

前置:carto 必须已经在跑(测量基准取 TF map->laser_link)。
⚠ 车前方留出 distance_cm + 1m 空地,开环走歪时横向可能甩几十 cm。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS3'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('rpm4_period_ms', default_value='20'),
        DeclareLaunchArgument('distance_cm', default_value='200.0'),
        DeclareLaunchArgument('speed_mps', default_value='0.16'),

        Node(
            package='ground_chassis_pkg',
            executable='chassis_bridge.py',
            name='chassis_bridge',
            output='screen',
            arguments=[
                '--port', LaunchConfiguration('port'),
                '--baud', LaunchConfiguration('baud'),
                # 测试脚本自己按 20Hz 发 cmd_vel,底盘侧超时兜底照常开
                '--chassis-timeout-ms', '500',
                '--rpm4-period-ms', LaunchConfiguration('rpm4_period_ms'),
            ],
        ),
        Node(
            package='ground_chassis_pkg',
            executable='wheel_health_monitor.py',
            name='wheel_health_monitor',
            output='screen',
        ),
        Node(
            package='ground_chassis_pkg',
            executable='straight_line_test.py',
            name='straight_line_test',
            output='screen',
            parameters=[{
                'distance_cm': LaunchConfiguration('distance_cm'),
                'speed_mps': LaunchConfiguration('speed_mps'),
            }],
        ),
    ])
