"""飞车难民识别 YOLO 节点(YOLOv5s + RK3588 NPU / RKNN)。

启动:
  yolo_detector   读 /dev/video0 -> NPU 推理 -> MJPEG 推流 + ~/detections

看视频流:    浏览器打开  http://<飞车IP>:8080/
看检测数据:  ros2 topic echo /yolo_detector/detections
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("infer_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("conf_thresh", default_value="0.25"),
        DeclareLaunchArgument("npu_core", default_value="auto"),
        DeclareLaunchArgument("enable_stream", default_value="true"),
        DeclareLaunchArgument("stream_port", default_value="8080"),
        # ROS 话题调试图默认关(视频流走 MJPEG,不必再占带宽)
        DeclareLaunchArgument("publish_debug", default_value="false"),

        Node(
            package="yolo_detector_pkg",
            executable="yolo_detector_node.py",
            name="yolo_detector",
            output="screen",
            parameters=[{
                "camera_device": LaunchConfiguration("camera_device"),
                "infer_rate_hz": LaunchConfiguration("infer_rate_hz"),
                "conf_thresh": LaunchConfiguration("conf_thresh"),
                "npu_core": LaunchConfiguration("npu_core"),
                "enable_stream": LaunchConfiguration("enable_stream"),
                "stream_port": LaunchConfiguration("stream_port"),
                "publish_debug": LaunchConfiguration("publish_debug"),
            }],
        ),
    ])
