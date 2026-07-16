"""陆空协同分区搜索 —— 飞车侧上板测试入口(阶段 2)。

起 xmachine_bridge + yolo_detector(视频源),**不碰底盘/建图/飞控**。用来验车侧 terminal
规划的巡航航点(UDP FC0A)能不能落到飞车本地、飞行节点状态(FC0B)能不能回传给 terminal,
以及摄像头画面能不能经 UDP(FC08)传到车、显示在 /slam 页的视频面板上。

⚠ **视频源在飞车这边**:车侧只是收流的一端。不起本 launch(或 with_video:=false),
   平板 /slam 页「机」那一路画面就一直是黑的 —— 这不是车侧的问题。

配对:车侧 car_launch/patrol_test.launch.py。

--- 为什么不直接用 comm_test.launch.py ---
comm_test 没有注入 ROS_DOMAIN_ID=1,桥会跑在默认域 0。此时 `ros2 topic echo` 看得到航点,
但**真正的飞行节点 wildlife_patrol_task_node 在域 1**,收不到 —— 等于测了个假的。
本 launch 与 my_launch/demo_ground.launch.py 一致,把桥放进域 1。

--- 测试步骤 ---
0. 前置:两板互 ping(飞车 .171 / 车 .161)、RMW fastrtps。车板跑 car_launch/patrol_test.launch.py。
   ⚠ 飞车板上手动跑 ros2 命令要先 `export ROS_DOMAIN_ID=1`,否则看不到本机节点。

1. 车→飞车 巡航航点(FC0A):
     [飞车] export ROS_DOMAIN_ID=1 && ros2 topic echo /wildlife/waypoints
     [车]   在 /slam 页按"开始"(车先走,3 秒后才发飞车)
     期望:收到 51x4 个 float。**首点应为 [0, 0, 0, 0]**(陆地转场起点,z=0)。

     ⚠⚠ 已知风险:car/follower_pkg/src/xmachine_bridge.cpp 的注释写着 FC0A"首点须 (0,0,120,0)"。
        本设计的首点 z=0(陆地段先地面行驶,到形态切换点才升空)。两侧桥都只转发不校验,
        所以这条约束来自飞行节点(队友 chenguanyi/ros2_ws,不在本仓库)。
        **先不装桨、不上电起飞**,只 echo 看数据。确认飞行节点拿到 z=0 是透传给 chassis_mux
        走地面底盘、而不是当起飞点,再往下测。这条不成立,陆地段就不能走 FC0A。

2. 飞车→车 状态回传(FC0B):
     [飞车] export ROS_DOMAIN_ID=1 && ros2 topic pub --once \
              --qos-durability transient_local --qos-reliability reliable \
              /wildlife/status std_msgs/msg/String "{data: '{\"phase\":\"IDLE\",\"ready\":true}'}"
     [车]   终端 /slam 页应能看到巡航状态(或 curl 127.0.0.1:8080/api/patrol/status)

3. 飞车→车 障碍触发重规划(FC03,真链路版):
     [飞车] export ROS_DOMAIN_ID=1 && ros2 topic pub --once \
              --qos-durability transient_local --qos-reliability reliable \
              /detected_obstacle std_msgs/msg/Float32MultiArray \
              "{data: [3.0, 0.1,-1.2, 2.0,-1.4, 3.9,-1.6, 0.8, 4.0]}"
     [车]   终端日志应打印"实测障碍已更新分界线: 3 顶点, 车搜索区 X 格, 飞车巡航区 Y 格"
     (只想在车板本地灌假障碍、不经飞车,用车侧 patrol_fake_obstacle.launch.py)

看不到数据先查:IP(飞车.171/车.161)、端口(车 bind 8890、飞车 bind 8891)、互 ping、
两侧桥终端的收发日志、以及 echo 前有没有 export ROS_DOMAIN_ID=1。

用法:  ros2 launch activity_control_pkg patrol_test.launch.py
       ros2 launch activity_control_pkg patrol_test.launch.py with_video:=false  # 只测航点不要视频
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 跨机走原生 UDP,与域无关;但桥要跟飞行节点(域 1)对话,必须先进域 1 再起节点。
    # 参数与 comm_test.launch.py / relief_drop_mission.launch.py 保持一致,改一处全都要改。
    bridge = Node(
        package="activity_control_pkg",
        executable="xmachine_bridge",
        name="xmachine_bridge",
        output="screen",
        parameters=[{
            "car_ip": "192.168.10.161",   # 车在路由器网段的固定地址
            "to_car_port": 8890,          # 车侧 bind 此口收 req/obst
            "from_car_port": 8891,        # 本节点 bind 此口收车的 done/waypoints
            "send_hz": 10.0,
            "resend_count": 30,
        }],
    )

    # YOLO 节点自带跨机视频发送(enable_udp_video 默认 true → 车 .161:8892),
    # 车侧 flycar_video_bridge 重组后给 /slam 页。视频走裸 UDP,与 ROS 域无关。
    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("yolo_detector_pkg"),
            "launch", "yolo_detector.launch.py")),
        condition=IfCondition(LaunchConfiguration("with_video")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "with_video", default_value="true",
            description="起 yolo_detector 把摄像头画面经 UDP 发给车(/slam 页「机」那一路)。"
                        "关了平板上就没有飞车画面",
        ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
        bridge,
        yolo,
    ])
