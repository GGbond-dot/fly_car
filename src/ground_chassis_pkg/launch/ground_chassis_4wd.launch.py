"""飞车地面【四驱】底盘 + 地空互斥仲裁。

与两驱版 ground_chassis.launch.py 的差别只有三处 —— 四轮差速对上层仍是差速,
v/w 语义不变,所以控制链结构完全沿用:

  1. ctrl_offset_x_cm  10.5 -> 0
     两驱是"前驱两轮+后万向轮",旋转中心在前驱动轮轴,而雷达在它后方 10.5cm,
     所以要把控制点平移过去。四驱**雷达就装在四轮几何中心**(= 旋转中心),
     两者重合,偏移归零即可,不需要再补。
     附带好处:雷达在旋转中心上,车拧头时 carto 位姿不再产生横向位移分量
     (两驱时雷达偏心 10.5cm,每次航向修正都会甩出一小段假位移,再被控制器
     当成位置误差去纠 —— 这个耦合在四驱上直接消失,直线段应更干净)。
  2. 新增 wheel_health_monitor
     四驱某个轮虚接触/打滑时车照跑只是跑偏,光看轨迹分不清是控制问题还是
     机械问题。这个节点把两者分开(详见脚本头注释)。
  3. wheel_base_m 用**等效轴距**
     四驱滑移转向的等效轴距大于几何轴距,尺子量的不对,得实测标 —— 这个值
     在固件侧 $SET,WHEEL,<轮径>,<轴距>,不在本 launch。

⚠ 以下标定值在换四驱后**全部作废**,必须重标:
   固件 $SET: WHEEL(轮径/等效轴距)、PID、FF(四轮前馈)
   本文件:  w_bias_rps、ki_w、straight_kp_w
   重标顺序见 straight_line_test.py 的结论表提示。

启动:
  chassis_bridge          $VW 下行 + $RPM4 四轮上行(/dev/ttyS3 @115200)
  diff_drive_controller   /target_position + /ground_enable -> /cmd_vel
  chassis_mux             按 /target_position 的 z 发布 /ground_enable 与 /flight_enable
  wheel_health_monitor    /chassis/wheel_rpm -> /chassis/wheel_diag
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS3'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('chassis_timeout_ms', default_value='500'),
        # 四轮 rpm 上报周期。编码器实测上限 50Hz,20ms 已是最快,再小固件也刷不出新数
        DeclareLaunchArgument('rpm4_period_ms', default_value='20'),
        DeclareLaunchArgument('z_threshold_cm', default_value='20.0'),
        DeclareLaunchArgument('land_height_cm', default_value='15.0'),
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
                '--rpm4-period-ms', LaunchConfiguration('rpm4_period_ms'),
            ],
        ),
        Node(
            package='ground_chassis_pkg',
            executable='diff_drive_controller',
            name='diff_drive_controller',
            output='screen',
            parameters=[{
                'kp_v': 1.0, 'v_max_mps': 0.2,
                'kp_w': 0.6, 'w_max_rps': 1.0,
                'straight_kp_w': 0.12, 'straight_w_max_rps': 0.10,
                'yaw_lpf_alpha': 0.12,
                'straight_yaw_deadband_deg': 2.0,
                'lookahead_dist_cm': 40.0,
                'v_min_mps': 0.14,
                'v_floor_gate_deg': 35.0,
                # ⚠ 四驱后左右恒定速度差是全新的量,两驱标的值不能用。
                # 先跑 test_straight_line.launch.py,结论表会直接给出建议值。
                'w_bias_rps': 0.0,
                'ki_w': 0.3, 'iw_limit_rps': 0.3,
                # 四驱去掉万向轮 = 去掉了拖距造成的纯滞后,收尾超调应显著减小。
                # kd_w 是当初为治万向轮甩头加的,四驱上先归 0,实测还摆再加回来。
                'kd_w': 0.0, 'yaw_rate_lpf_alpha': 0.5,
                'w_slew_rps2': 0.5,
                'w_min_rps': 0.45,
                'log_csv_path': '/home/orangepi/kian_flycar/test_log/',
                'align_gate_deg': 60.0,
                'pos_tol_cm': 5.0, 'yaw_tol_deg': 3.0,
                'ground_enable_default': False,
                # 四驱旋转中心 = 几何中心,不再是前驱动轮轴 → 偏移归零
                'ctrl_offset_x_cm': 0.0,
                'ctrl_offset_y_cm': 0.0,
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
        Node(
            package='ground_chassis_pkg',
            executable='wheel_health_monitor.py',
            name='wheel_health_monitor',
            output='screen',
            parameters=[{
                # ppr=255 @50Hz 上报,巡航 66rpm 时量化步进约 11.8rpm。
                # 阈值给 2 个步进,再小就是在报量化噪声。
                'side_mismatch_rpm': 24.0,
                'stall_rpm': 6.0,
                'hold_s': 0.4,
            }],
        ),
    ])
