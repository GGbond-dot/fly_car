"""飞车地面差速底盘 + 地空互斥仲裁。

启动:
  chassis_bridge          SR5E1E3 $VW 串口桥(/dev/ttyS6 @115200)
  diff_drive_controller   /target_position + /ground_enable -> /cmd_vel
  chassis_mux             按 /target_position 的 z 发布 /ground_enable 与 /flight_enable

⚠ 飞控侧 pid_control_pkg 需配合订阅 /flight_enable(已加标志位):
  地面态(z≤阈值)时 chassis_mux 置 /flight_enable=false,pid 停发 /target_velocity,
  保证车跑时飞控不动、飞控起飞时轮子停(两套控制链互斥)。

本 launch 只管地面链 + 仲裁;飞控链(pid/uart)、建图、任务编排(coverage_mission)各自启动。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS6'),
        DeclareLaunchArgument('baud', default_value='115200'),
        # 跟随/任务态建议开底盘侧超时兜底(静默自动刹停)
        DeclareLaunchArgument('chassis_timeout_ms', default_value='500'),
        DeclareLaunchArgument('z_threshold_cm', default_value='20.0'),
        # 降落判据:实测 /height 低于此才允许从飞控切回地面(防止半空中切换掉落)
        DeclareLaunchArgument('land_height_cm', default_value='15.0'),
        # 起飞/降落切换前的稳定停顿:起飞前停 settle 秒再升,落到 land_height 后停 settle 秒再跑
        DeclareLaunchArgument('settle_s', default_value='2.0'),

        Node(
            package='ground_chassis_pkg',
            executable='chassis_bridge.py',
            name='chassis_bridge',
            output='screen',
            arguments=[
                '--port', LaunchConfiguration('port'),
                '--baud', LaunchConfiguration('baud'),
                '--chassis-timeout-ms', LaunchConfiguration('chassis_timeout_ms'),
            ],
        ),
        Node(
            package='ground_chassis_pkg',
            executable='diff_drive_controller',
            name='diff_drive_controller',
            output='screen',
            parameters=[{
                'kp_v': 1.0, 'v_max_mps': 0.4,
                'kp_w': 1.5, 'w_max_rps': 1.0,
                'align_gate_deg': 45.0,
                'pos_tol_cm': 5.0, 'yaw_tol_deg': 8.0,
                'ground_enable_default': False,  # 等 mux 使能,默认不动
            }],
        ),
        Node(
            package='ground_chassis_pkg',
            executable='chassis_mux',
            name='chassis_mux',
            output='screen',
            parameters=[{
                'z_threshold_cm': LaunchConfiguration('z_threshold_cm'),
                'land_height_cm': LaunchConfiguration('land_height_cm'),
                'settle_s': LaunchConfiguration('settle_s'),
            }],
        ),
    ])
