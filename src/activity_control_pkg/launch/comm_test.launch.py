"""跨机通信自测入口(飞车侧)—— 只起 xmachine_bridge,不碰底盘/建图/任务。

用来单独验证起飞前的跨机握手 + 折线转发是否通(原生 UDP,两板都在域 0)。
配对:车侧 car/follower_pkg/launch/comm_test.launch.py。

--- 测试步骤(两板都先 build 好、都跑各自的 comm_test)---
1. 飞车→车 叫车:
     [飞车] ros2 topic pub --once /resupply_request std_msgs/msg/Bool "{data: true}"
     [车]   ros2 topic echo /resupply_request        # 应收到 data: true
2. 车→飞车 补给完成:
     [车]   ros2 topic pub --once /resupply_done std_msgs/msg/Bool "{data: true}"
     [飞车] ros2 topic echo /resupply_done           # 应收到 data: true
3. 飞车→车 折线障碍(发一次,桥会 burst 重发扛丢包):
     [飞车] ros2 topic pub --once /detected_obstacle std_msgs/msg/Float32MultiArray \
              "{data: [2, 0.0,0.0, 1.0,0.0, 0.5, 1.0]}"
     [车]   ros2 topic echo /detected_obstacle       # 应收到同样的数组
4. 飞车→车 YOLO 识别标志(带类别;连发 2 帧过去抖,cls=1→rescuee2):
     [飞车] ros2 topic pub -r 5 /yolo_detector/detections std_msgs/msg/Float32MultiArray \
              "{data: [1, 0.9, 0,0, 10,10]}"          # 模拟 YOLO,-r 5 连发
     [车]   ros2 topic echo /rescuee_detected         # 应收到 data: 2(rescuee2)
5. 车(terminal)→飞车 确认:
     [车]   ros2 topic pub --once /terminal_confirm std_msgs/msg/Bool "{data: true}"
     [飞车] ros2 topic echo /terminal_confirm         # 应收到 data: true

两侧 xmachine_bridge 终端会打日志(收到 REQ/DONE/OBST/RESCUEE/CONFIRM、开始重发)。看不到就先查:
IP(飞车.171/车.161)、端口(车 bind 8890、飞车 bind 8891)、两板能否互 ping。

用法:  ros2 launch activity_control_pkg comm_test.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="xmachine_bridge",
            name="xmachine_bridge",
            output="screen",
            parameters=[{
                "car_ip": "192.168.10.161",   # 车在路由器网段的固定地址
                "to_car_port": 8890,          # 车侧 bind 此口收 req/obst
                "from_car_port": 8891,        # 本节点 bind 此口收车的 done
                "send_hz": 10.0,
                "resend_count": 30,
            }],
        ),
    ])
