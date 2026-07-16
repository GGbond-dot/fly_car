"""飞车难民识别 YOLO 节点(YOLOv5s + RK3588 NPU / RKNN)。

启动:
  yolo_detector   读 /dev/video0 -> NPU 推理 -> MJPEG 推流 + ~/detections

看视频流:    浏览器打开  http://<飞车IP>:8080/
看检测数据:  ros2 topic echo /yolo_detector/detections
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_dir = os.path.join(
        get_package_share_directory("yolo_detector_pkg"), "models")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=os.path.join(
                model_dir,
                "yoloqian_formal_best_rk3588_fp16_single_output_640.rknn"),
        ),
        DeclareLaunchArgument(
            "classes_path",
            default_value=os.path.join(model_dir, "classes.txt"),
        ),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("infer_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("conf_thresh", default_value="0.25"),
        DeclareLaunchArgument("npu_core", default_value="auto"),
        DeclareLaunchArgument("rotate", default_value="cw90"),
        DeclareLaunchArgument("enable_stream", default_value="true"),
        DeclareLaunchArgument("stream_port", default_value="8080"),
        # ROS 话题调试图默认关(视频流走 MJPEG,不必再占带宽)
        DeclareLaunchArgument("publish_debug", default_value="false"),
        # 跨机视频:标注帧裸 UDP(FC08)发给车,车侧重组 -> 平板"机"看流。默认开。
        DeclareLaunchArgument("enable_udp_video", default_value="true"),
        DeclareLaunchArgument("car_ip", default_value="192.168.10.161"),
        DeclareLaunchArgument("video_port", default_value="8892"),

        Node(
            package="yolo_detector_pkg",
            executable="yolo_detector_node.py",
            name="yolo_detector",
            output="screen",
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "classes_path": LaunchConfiguration("classes_path"),
                "camera_device": LaunchConfiguration("camera_device"),
                "infer_rate_hz": LaunchConfiguration("infer_rate_hz"),
                "conf_thresh": LaunchConfiguration("conf_thresh"),
                "npu_core": LaunchConfiguration("npu_core"),
                "rotate": LaunchConfiguration("rotate"),
                "enable_stream": LaunchConfiguration("enable_stream"),
                "stream_port": LaunchConfiguration("stream_port"),
                "publish_debug": LaunchConfiguration("publish_debug"),
                "enable_udp_video": LaunchConfiguration("enable_udp_video"),
                "car_ip": LaunchConfiguration("car_ip"),
                "video_port": LaunchConfiguration("video_port"),
            }],
        ),
    ])
