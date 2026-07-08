"""YOLO + 跨机桥 联测入口(飞车侧)—— 不碰底盘/建图。

用来验证"真实 YOLO 识别 → 飞车桥 → 车"这条链,以及视频流:
  yolo_detector    读 /dev/video0 → NPU 推理 → /yolo_detector/detections + MJPEG(:8080)
  xmachine_bridge  订 /yolo_detector/detections,识别到 rescuee 就发 UDP 给车;收车 confirm

配对:车侧 car/follower_pkg/launch/comm_test.launch.py(只起桥)。

--- 测试步骤 ---
1. 视频 + 识别:浏览器开 http://192.168.10.171:8080/,镜头对准难民,应出画面 + 识别框。
2. 识别标志到车(飞车看到 rescuee 时):
     [车] ros2 topic echo /rescuee_detected      # 应收到 data: 1 或 2(类别)
3. terminal(或手动模拟)确认:
     [车]   ros2 topic pub --once /terminal_confirm std_msgs/msg/Bool "{data: true}"
     [飞车] ros2 topic echo /terminal_confirm     # 应收到 data: true

两板都不设 ROS_DOMAIN_ID(默认域 0)。跑不通先查:互 ping、/dev/video0、rknnlite。

用法:  ros2 launch my_launch yolo_comm_test.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def _include(package_name: str, filename: str) -> IncludeLaunchDescription:
    share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename))
    )


def generate_launch_description():
    yolo = _include("yolo_detector_pkg", "yolo_detector.launch.py")
    bridge = _include("activity_control_pkg", "comm_test.launch.py")  # 只有 xmachine_bridge
    return LaunchDescription([yolo, bridge])
