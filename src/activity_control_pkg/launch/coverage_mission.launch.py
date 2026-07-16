"""平地遍历 + 遇障起飞 任务编排入口。

coverage_mission_node 一个进程内拼:航点队列(RouteTargetPublisher)+ 覆盖生成器 + 障碍决策。
⚠ 场地边界 area_*、行距、巡航/越障高度、逼近阈值都是占位默认值,必须按实际场地标定
  (见 docs/coverage_flyover_mission_design.md §六)。obstacle_detector 需另行启动。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="coverage_mission_node",
            name="coverage_mission",
            output="screen",
            parameters=[{
                # --- RouteTargetPublisher ---
                "map_frame": "map",
                "laser_link_frame": "laser_link",
                "output_topic": "/target_position",
                "position_tolerance_cm": 6.0,
                "yaw_tolerance_deg": 5.0,
                "height_tolerance_cm": 6.0,
                # --- 覆盖生成器(⚠ 占位,按场地改)---
                # 本 launch 是单机弓字形障碍自测:mode=boustrophedon + auto_start 开机自生成。
                # (飞车任务实走 L 形 mode=l_path,由地面站信号触发,见 relief/coverage 任务 launch)
                "mode": "boustrophedon",
                "auto_start": True,
                "area_x_min_cm": 0.0,
                "area_x_max_cm": 500.0,
                "area_y_min_cm": 0.0,
                "area_y_max_cm": 300.0,
                "grid_cell_cm": 100.0,   # 与 web 地图 1m 网格一致
                "lane_cells": 1.0,       # 行距 = 格子 × 1
                "cruise_z_cm": 4.0,
                "start_delay_s": 2.0,
                # --- 障碍决策(墙逼近→原地起飞+全局 z 覆盖)---
                "approach_threshold_m": 0.6,  # 以车为心半径 0.6m
                "flyover_z_cm": 100.0,
                "single_shot": True,
            }],
        )
    ])
