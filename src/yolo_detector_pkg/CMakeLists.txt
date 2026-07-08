cmake_minimum_required(VERSION 3.8)
project(yolo_detector_pkg)

find_package(ament_cmake REQUIRED)

# 纯 Python 节点(rclpy + rknnlite + opencv 均为运行时依赖,不在此 find_package)。

# YOLO 识别节点 + 共用推理模块
install(PROGRAMS
  scripts/yolo_detector_node.py
  scripts/rknn_yolov5.py
  scripts/mjpeg_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# RKNN 模型与类别表(装到 share,节点默认从这里加载)
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py")

ament_package()
