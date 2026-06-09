from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_detector_pkg',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[{
                # ── 话题 / 坐标系 ──
                'scan_topic': '/scan',
                'enable_topic': '/obstacle_detect_enable',
                'map_frame': 'map',
                'laser_link_frame': 'laser_link',

                # ── 墙期望出现的世界 ROI (map 系, 单位 m)，按场地标定 ──
                # 墙在车前方，沿 x 取一段、y 取车两侧的宽度
                'roi_x_min_m':  0.20,
                'roi_x_max_m':  6.00,
                'roi_y_min_m': -3.00,
                'roi_y_max_m':  3.00,

                # ── 折线(Split-and-Merge)拟合 ──
                'chain_break_dist_m': 0.20,    # 相邻点超此距离断链
                'min_chain_points': 15,        # 点链最少点数
                'split_threshold_m': 0.05,     # 点到段超此距离则劈开
                'merge_collinear_deg': 10.0,   # 相邻段转角小于此值则合并
                'min_total_length_m': 0.50,    # 折线总长下限

                # ── TF 查询超时 ──
                'tf_timeout_sec': 0.05,

                # ── 调参旁路 ──
                'publish_debug_points': True,
                'debug_points_topic': '/obstacle_debug_points',
            }],
        )
    ])
