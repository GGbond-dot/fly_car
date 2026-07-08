#!/usr/bin/env python3
"""飞车难民识别 YOLO 节点(YOLOv5s + RK3588 NPU / RKNN)。

读 /dev/video0 摄像头 -> RKNN NPU 推理 -> 发布检测结果 + (可选)标注调试图。
摄像头俯仰由舵机2控制(地面 120°/飞行 180°,见 servo_test.py);本节点只管识别。

视频流(主用途):内置 MJPEG over HTTP 推流,局域网浏览器直接看标注画面。
  手机/电脑打开  http://<飞车IP>:<stream_port>/   即可(默认端口 8080),零依赖。

发布:
  ~/detections  std_msgs/Float32MultiArray
      data = [n,  然后每目标 6 个: cls_id, conf, x1, y1, x2, y2 (原始帧像素) ...]
      layout.dim[0].label = "rescuee_detections", stride=6
  ~/image_annotated  sensor_msgs/CompressedImage (jpeg)   仅 publish_debug=true 时
      用 cv2.imencode 直接压 jpeg,免 cv_bridge 依赖,便于 rqt 看

参数:
  model_path        .rknn 路径(默认取本包 share/models 下)
  classes_path      classes.txt 路径(默认同上)
  camera_device     摄像头设备,默认 /dev/video0(video1 是 metadata 别用)
  frame_width/height  采集分辨率,默认 640x480
  infer_rate_hz     推理频率,默认 15
  conf_thresh/nms_thresh  置信度/NMS 阈值
  publish_debug     是否发标注 jpeg 到 ROS 话题,默认 false(视频流走 MJPEG,不必开)
  jpeg_quality      标注 jpeg 质量 1-100,默认 70(MJPEG 流与 ROS 话题共用)
  npu_core          NPU 核: auto/0/1/2/012,默认 auto
  enable_stream     是否开 MJPEG over HTTP 推流,默认 true
  stream_host       监听地址,默认 0.0.0.0
  stream_port       推流端口,默认 8080
"""

import os
import sys

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, MultiArrayDimension
    from sensor_msgs.msg import CompressedImage
except ImportError:
    print("ERROR: 需在 ROS2 环境运行,先 source /opt/ros/<distro>/setup.bash", file=sys.stderr)
    raise

import cv2

# 同包内共用推理模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rknn_yolov5 import RknnYolov5, draw_detections, load_class_names  # noqa: E402
from mjpeg_server import MjpegServer  # noqa: E402


def default_model_dir():
    """优先用安装后的 share/models;找不到就回退到源码树 models/。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("yolo_detector_pkg"), "models")
    except Exception:
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")


_CORE_MAP = None


def resolve_core_mask(name):
    global _CORE_MAP
    from rknnlite.api import RKNNLite
    if _CORE_MAP is None:
        _CORE_MAP = {
            "auto": RKNNLite.NPU_CORE_AUTO,
            "0": RKNNLite.NPU_CORE_0,
            "1": RKNNLite.NPU_CORE_1,
            "2": RKNNLite.NPU_CORE_2,
            "012": RKNNLite.NPU_CORE_0_1_2,
        }
    return _CORE_MAP.get(str(name).lower(), RKNNLite.NPU_CORE_AUTO)


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")
        md = default_model_dir()
        self.declare_parameter("model_path",
                               os.path.join(md, "yoloqian_formal_best_rk3588_int8.rknn"))
        self.declare_parameter("classes_path", os.path.join(md, "classes.txt"))
        self.declare_parameter("camera_device", "/dev/video0")
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("infer_rate_hz", 15.0)
        self.declare_parameter("conf_thresh", 0.25)
        self.declare_parameter("nms_thresh", 0.45)
        self.declare_parameter("img_size", 640)
        self.declare_parameter("publish_debug", False)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("npu_core", "auto")
        self.declare_parameter("enable_stream", True)
        self.declare_parameter("stream_host", "0.0.0.0")
        self.declare_parameter("stream_port", 8080)

        model_path = self.get_parameter("model_path").value
        classes_path = self.get_parameter("classes_path").value
        self.device = self.get_parameter("camera_device").value
        self.width = int(self.get_parameter("frame_width").value)
        self.height = int(self.get_parameter("frame_height").value)
        rate = float(self.get_parameter("infer_rate_hz").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"model not found: {model_path}")
        self.class_names = load_class_names(classes_path)

        self.model = RknnYolov5(
            model_path, self.class_names,
            img_size=int(self.get_parameter("img_size").value),
            conf_thresh=float(self.get_parameter("conf_thresh").value),
            nms_thresh=float(self.get_parameter("nms_thresh").value),
            core_mask=resolve_core_mask(self.get_parameter("npu_core").value),
            logger=lambda m: self.get_logger().info(m),
        )

        self.cap = self._open_camera()

        self.det_pub = self.create_publisher(Float32MultiArray, "~/detections", 10)
        self.img_pub = None
        if self.publish_debug:
            self.img_pub = self.create_publisher(CompressedImage, "~/image_annotated", 5)

        # 是否需要标注帧(MJPEG 推流 或 ROS 调试话题任一开启)
        self.stream = None
        if bool(self.get_parameter("enable_stream").value):
            host = self.get_parameter("stream_host").value
            port = int(self.get_parameter("stream_port").value)
            self.stream = MjpegServer(host, port, title="flycar rescuee").start()
            self.get_logger().info(f"MJPEG 推流: http://{host}:{port}/  (浏览器直接看)")
        self.want_annotated = self.stream is not None or self.img_pub is not None

        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.on_timer)
        self._miss = 0
        self.get_logger().info(
            f"yolo_detector up: {self.device} {self.width}x{self.height} "
            f"@{rate:.0f}Hz classes={self.class_names} "
            f"stream={self.stream is not None} ros_debug={self.publish_debug}")

    def _open_camera(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(self.device)  # 回退默认后端
        if not cap.isOpened():
            raise RuntimeError(f"cannot open camera: {self.device}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        return cap

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self._miss += 1
            if self._miss % 30 == 1:
                self.get_logger().warn(f"camera read failed x{self._miss}")
            return
        self._miss = 0

        dets = self.model.infer(frame)
        self._publish_detections(dets)

        if self.want_annotated:
            annotated = draw_detections(frame, dets, self.class_names)
            ok, buf = cv2.imencode(
                ".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                jpeg = buf.tobytes()
                if self.stream is not None:
                    self.stream.update_frame(jpeg)
                if self.img_pub is not None:
                    self._publish_debug_image(jpeg)

    def _publish_detections(self, dets):
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = "rescuee_detections"  # 每目标 6 元素: cls,conf,x1,y1,x2,y2
        dim.size = len(dets)
        dim.stride = 6
        msg.layout.dim = [dim]
        data = [float(len(dets))]
        for x1, y1, x2, y2, conf, cls in dets:
            data += [float(cls), float(conf), x1, y1, x2, y2]
        msg.data = data
        self.det_pub.publish(msg)

    def _publish_debug_image(self, jpeg_bytes):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = jpeg_bytes
        self.img_pub.publish(msg)

    def destroy_node(self):
        try:
            if self.stream is not None:
                self.stream.stop()
            if self.cap is not None:
                self.cap.release()
            self.model.release()
        finally:
            super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    code = 0
    try:
        node = YoloDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"yolo_detector fatal: {exc}", file=sys.stderr)
        code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
