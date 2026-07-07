"""飞车地面差速底盘 + 地空互斥仲裁。

启动:
  chassis_bridge          SR5E1E3 $VW 串口桥(/dev/ttyS3 @115200)
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
        DeclareLaunchArgument('port', default_value='/dev/ttyS3'),
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
                # 上层为纯 P carrot-chasing;换电机/驱动后底盘响应变快,压低航向增益抑制画龙震荡。
                'kp_v': 1.0, 'v_max_mps': 0.2,    # pure-pursuit 巡航速度(需 >起步阈值~0.13,配 v_min 地板)
                'kp_w': 0.6, 'w_max_rps': 0.8,    # 转向到前视点的增益/上限;移动中转弯轮子在滚,不卡 stick-slip
                # pure-pursuit:追前方 lookahead 处的前视点(消近点方位角超敏 + v 塌陷卡顿),自动圆角。
                # 需 route 端 lookahead_count>0 才生效。大→更顺更抄近路;小→更贴线。
                'lookahead_dist_cm': 30.0,
                # 线速度地板:巡航/直行时 v 不低于此(>起步阈值~0.13),防轮子掉死区卡顿。
                'v_min_mps': 0.14,
                # 航向环积分:自适应补偿左轮>右轮的(缓变)速度差,让直行不跑偏。上板从 0.3 起调:
                # 跑偏纠不回来→加大;走直后左右摆/纠过头→减小。
                'ki_w': 0.3, 'iw_limit_rps': 0.3,
                # 航向环微分阻尼(治万向轮拖距=纯滞后的转向收尾超调/摆动):w -= kd_w*yaw_rate。
                # 弧线测试先设 0:kd 惩罚 yaw_rate,会跟"持续转弯"的弧线对着干。直线收尾才需要它。
                'kd_w': 0.0, 'yaw_rate_lpf_alpha': 0.5,
                # w 斜率限制(禁止猛打方向,减小对滞后系统的激励):|dw/dt| 上限 rad/s^2。
                # 起调 3.0(约 0.23s 到 w_max)。太肉→加大;甩头太猛→减小。设 0=不限。
                'w_slew_rps2': 3.0,
                # align 原地拧最小转速(破底盘起步死区,否则 yaw 拧到最后几度蹭不动、卡死不推进)。
                # 起调 0.45(≈w_max);还拧不到位→加大或加大 w_max;到位后抖→减小。设 0=关。仅 align 用。
                'w_min_rps': 0.45,
                # 调参采集:目录(以 / 结尾)则自动按时间戳命名 ddc_日期_时间.csv,每跑一次一个文件;
                # 留空=关。板上目录: ~/kian_flycar/test_log/
                'log_csv_path': '/home/orangepi/kian_flycar/test_log/',
                'align_gate_deg': 60.0,           # 放大→角点边走边转走圆弧,少纯原地拧(拧最易抖)
                # yaw_tol 必须 ≤ route_target_publisher 的 yaw_tolerance_deg(默认 5°),否则控制器
                # 提前收工(8°)而 route 还没判到达(5°)→ 6~8° 夹缝里俩节点互等、原地卡死。设 3° 更严。
                'pos_tol_cm': 5.0, 'yaw_tol_deg': 3.0,
                'ground_enable_default': False,  # 等 mux 使能,默认不动
                # 控制点偏移(雷达→前驱动轮轴中点,车体系)。前驱动轮轴在雷达前方 10.5cm、横向 0
                'ctrl_offset_x_cm': 10.5,
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
    ])
