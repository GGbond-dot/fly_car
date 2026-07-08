"""灾区两次投放 任务编排入口(飞车侧)。

relief_drop_mission_node 一个进程内拼:航点队列(RouteTargetPublisher)+ 任务状态机
(MissionSequencer)。坐标是现场标定值,右转 = yaw -90(REP-103:-y 方向)。

跨机通信(补给握手 + 折线障碍):
  - 两板都没设 ROS_DOMAIN_ID(都在域 0)、路由器又挡 DDS 多播 —— 所以跨机一律走原生 UDP,
    完全不碰 DDS 发现,两机 scan/tf/map 天然隔离(不再用 domain_bridge / fastdds peers)。
  - xmachine_bridge(本 launch 起)本地 ↔ UDP 翻译:
      /resupply_request、/detected_obstacle → UDP 发车(.161)
      车的 UDP done → 本地 /resupply_done
    车侧对端:car/follower_pkg launch 里的 xmachine_bridge。

依赖的其它节点需另行启动:diff_drive_controller / chassis_bridge(含 /servo_cmd 转发)
  / chassis_mux / pid_control_pkg / uart_to_stm32 / yolo_detector / 建图 / obstacle_detector。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # --- 任务编排节点 ---
        Node(
            package="activity_control_pkg",
            executable="relief_drop_mission_node",
            name="relief_drop_mission",
            output="screen",
            parameters=[{
                # --- RouteTargetPublisher ---
                "map_frame": "map",
                "laser_link_frame": "laser_link",
                "output_topic": "/target_position",
                "position_tolerance_cm": 9.0,
                "yaw_tolerance_deg": 5.0,
                "air_z_tol_cm": 8.0,
                "ground_z_tol_cm": 30.0,

                # --- 任务高度 ---
                "z_ground_cm": 4.0,
                "flight_z_cm": 100.0,      # 原地起飞/巡航高度
                "z_drop2_cm": 50.0,        # 难民2 悬停投放高度(空中,不落地)
                "z_forward_cm": 50.0,      # 投货2 后前进平飞高度(默认=z_drop2,不额外爬升)
                "forward_after_drop2_cm": 100.0,  # 投货2 后沿机头前进距离,到点再垂直降落

                # --- 写死位置 map 坐标(现场标定,右转=yaw -90)---
                "fwd_x_cm": 245.0, "fwd_y_cm": 0.0, "fwd_yaw_deg": 0.0,      # YOLO 看到货处,纯路径整形
                "drop1_x_cm": 265.0, "drop1_y_cm": -30.0, "drop1_yaw_deg": -90.0,  # 右转走弧到此,投货1
                "takeoff_x_cm": 265.0, "takeoff_y_cm": -97.0, "takeoff_yaw_deg": -90.0,  # 起飞点;飞行段全程保持 -90
                "fly_waypoints": [-8.0, -270.0],   # [x,y,...] cm,最后一个 = 难民2 上方(投货2 下降点)

                # --- 摄像头舵机(舵机2:地面 120 / 飞行 180)---
                "camera_servo_index": 2,
                "camera_ground_deg": 120,
                "camera_flight_deg": 180,

                # --- 投货舵机(舵机1,实测:倒货180 / 初始复位90)---
                "servo_drop_index": 1,
                "servo_open_deg": 180,
                "servo_close_deg": 90,
                "t_drop_s": 1.5,
                "servo_cmd_topic": "/servo_cmd",

                # --- 补给握手 + 视觉握手(本地话题,经 xmachine_bridge 走 UDP 跨机)---
                "resupply_topic": "/resupply_done",             # 车→飞车:补给完成、已退开
                "resupply_request_topic": "/resupply_request",  # 飞车→车:到起飞点,叫车过来
                "terminal_confirm_topic": "/terminal_confirm",  # 车(terminal)→飞车:放行右转投货
                "wait_terminal_confirm": True,                  # 到检测点等 terminal 确认才右转投货

                # --- 启动门:起来后挂 INIT,等 terminal 语音"开始救援"(经 xmachine_bridge FC07)才开跑 ---
                "mission_start_topic": "/mission_start",        # 车(terminal)→飞车:开始救援
                "wait_mission_start": True,                     # False=延时后自动开跑(纯调试)

                "start_delay_s": 2.0,
            }],
        ),

        # --- 跨机信号原生 UDP 桥 ---
        Node(
            package="activity_control_pkg",
            executable="xmachine_bridge",
            name="xmachine_bridge",
            output="screen",
            parameters=[{
                "car_ip": "192.168.10.161",   # 车在路由器网段的固定地址
                "to_car_port": 8890,          # 车侧 bind 此口收 req/obst/rescuee
                "from_car_port": 8891,        # 本节点 bind 此口收车的 done/confirm
                "send_hz": 10.0,
                "resend_count": 30,           # 每次事件重发 ~3s,扛 UDP 丢包
                "yolo_detections_topic": "/yolo_detector/detections",
                "yolo_conf_thresh": 0.25,     # 判"识别到 rescuee"的最低置信度
                "yolo_debounce_frames": 2,    # 连续几帧命中才算(去抖)
            }],
        ),
    ])
