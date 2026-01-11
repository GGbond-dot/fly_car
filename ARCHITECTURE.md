# Fly Car 系统架构图

## 系统整体架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         Fly Car System                          │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────┐     ┌──────────────────┐     ┌──────────────┐
│   蓝牙遥控器    │────▶│  Bluetooth Node  │────▶│              │
└─────────────────┘     └──────────────────┘     │              │
                                                  │              │
┌─────────────────┐     ┌──────────────────┐     │              │
│  激光雷达传感器 │────▶│  Bluesea2 Node   │────▶│              │
└─────────────────┘     └──────────────────┘     │              │
                                                  │              │
┌─────────────────┐     ┌──────────────────┐     │   ROS2 Core  │
│   OpenMV 摄像头 │────▶│ OpenMV Bridge    │────▶│   (DDS 中间件) │
└─────────────────┘     └──────────────────┘     │              │
                                                  │              │
                        ┌──────────────────┐     │              │
                        │  Cartographer    │◀───▶│              │
                        │  (SLAM 建图)     │     │              │
                        └──────────────────┘     │              │
                                                  └───────┬──────┘
                                                          │
                        ┌─────────────────────────────────┼─────────────┐
                        │                                 │             │
                        ▼                                 ▼             ▼
            ┌─────────────────────┐         ┌──────────────────┐  ┌─────────────┐
            │ Activity Control    │         │  PID Controller  │  │   TF2       │
            │ (路径规划)          │─────▶  │  (位置控制)      │  │ (坐标变换)  │
            └─────────────────────┘         └──────────────────┘  └─────────────┘
                                                        │
                                                        ▼
                                            ┌──────────────────┐
                                            │  Uart to STM32   │
                                            │  (串口通信)      │
                                            └─────────┬────────┘
                                                      │
                                                      ▼
                                            ┌──────────────────┐
                                            │   STM32 MCU      │
                                            │  (电机控制器)    │
                                            └─────────┬────────┘
                                                      │
                        ┌─────────────────────────────┼────────────────┐
                        │                             │                │
                        ▼                             ▼                ▼
                ┌──────────────┐          ┌──────────────┐   ┌──────────────┐
                │  左前轮电机  │          │  右前轮电机  │   │  其他执行器  │
                └──────────────┘          └──────────────┘   └──────────────┘
                        │                             │                │
                        └─────────────────────────────┴────────────────┘
```

## 数据流向图

```
传感器层
    │
    ├─ 激光雷达 ──▶ [/scan] ──▶ Cartographer ──▶ [/map, /tf]
    │
    ├─ OpenMV ────▶ [/camera_data] ──▶ OpenMV Bridge
    │
    └─ 蓝牙 ───────▶ [/bluetooth_data] ──▶ Bluetooth Node

定位与地图层
    │
    ├─ Cartographer ──▶ [/map]
    │
    └─ TF2 ───────────▶ [/tf] (map ↔ laser_link 变换)

规划与控制层
    │
    ├─ Activity Control ──▶ [/target_position] ──▶ PID Controller
    │
    └─ PID Controller ────▶ [/target_velocity] ──▶ Uart to STM32

通信与执行层
    │
    └─ Uart to STM32 ─────▶ STM32 ──▶ 电机驱动
            │
            └──────────◀── STM32 ──▶ [/height, /is_st_ready]
```

## 话题 (Topic) 关系图

```
/scan ─────────────────▶ (Cartographer)
                              │
                              ├──▶ /map
                              └──▶ /tf

/target_position ──────▶ (PID Controller)
                              │
                              └──▶ /target_velocity

/target_velocity ──────▶ (Uart to STM32)
/velocity_map ─────────▶
/bluetooth_data ───────▶      │
                              ├──▶ /height
                              ├──▶ /is_st_ready
                              └──▶ /mission_step

/height ───────────────▶ (Activity Control)
/tf ───────────────────▶
```

## 坐标系层次结构

```
map (全局地图坐标系)
 │
 └─ base_link (机器人基座坐标系)
     │
     ├─ laser_link (激光雷达坐标系)
     │
     └─ camera_link (摄像头坐标系)

注：本系统主要使用 map 到 laser_link 的直接变换。
在完整的 ROS2 导航系统中，通常会包含 odom（里程计）坐标系作为中间层。
```

## 控制回路

```
┌──────────────┐
│  目标位置    │ (x_target, y_target, yaw_target)
└──────┬───────┘
       │
       ▼
┌──────────────────────────────────────┐
│  计算误差                            │
│  e_x = x_target - x_current          │
│  e_y = y_target - y_current          │
│  e_yaw = yaw_target - yaw_current    │
└──────┬───────────────────────────────┘
       │
       ▼
┌──────────────────────────────────────┐
│  PID 控制器                          │
│  u_x = Kp*e_x + Ki*∫e_x + Kd*de_x   │
│  u_y = Kp*e_y + Ki*∫e_y + Kd*de_y   │
│  u_yaw = Kp*e_yaw + ...              │
└──────┬───────────────────────────────┘
       │
       ▼
┌──────────────────────────────────────┐
│  速度命令                            │
│  (v_x, v_y, ω)                       │
└──────┬───────────────────────────────┘
       │
       ▼
┌──────────────────────────────────────┐
│  串口发送到 STM32                     │
└──────┬───────────────────────────────┘
       │
       ▼
┌──────────────────────────────────────┐
│  电机执行                            │
└──────┬───────────────────────────────┘
       │
       ▼
┌──────────────────────────────────────┐
│  传感器反馈当前位置                  │
│  (x_current, y_current, yaw_current) │
└──────┬───────────────────────────────┘
       │
       └─────────────────────────────────┐
                                         │
       ┌─────────────────────────────────┘
       │
       ▼
   [回到计算误差]
```

## 软件包依赖关系

```
pid_control_pkg
    ├── rclcpp (ROS2 C++ 客户端库)
    ├── std_msgs (标准消息类型)
    ├── geometry_msgs (几何消息类型)
    └── tf2 (坐标变换库)

activity_control_pkg
    ├── rclcpp
    ├── std_msgs
    ├── geometry_msgs
    ├── tf2_ros
    └── angles (角度处理工具)

uart_to_stm32
    ├── rclcpp
    ├── geometry_msgs
    ├── std_msgs
    ├── tf2
    ├── serial_comm (自定义串口库)
    └── Eigen3 (线性代数库)

bluetooth
    ├── rclcpp
    ├── std_msgs
    ├── sensor_msgs
    └── serial_comm

car_driver
    ├── rclcpp
    ├── std_msgs
    └── geometry_msgs

bluesea2
    ├── 激光雷达 SDK
    └── ROS2 接口

my_carto_pkg
    └── Google Cartographer
```

## 启动文件层次

```
total.launch.py (完整系统启动)
    │
    ├──▶ uart_to_stm32.launch.py
    │       └── uart_to_stm32_node
    │
    ├──▶ position_pid_controller.launch.py
    │       └── pid_controller_node
    │
    ├──▶ route_target_publisher.launch.py
    │       └── route_target_publisher_node
    │
    ├──▶ fly_carto.launch.py
    │       └── cartographer_node
    │
    └──▶ car_drive.launch.py
            └── car_driver_node
```

## 关键参数配置

```
PID 控制器参数:
    pid_x_kp: 0.3      # X轴比例增益
    pid_x_ki: 0.0      # X轴积分增益
    pid_x_kd: 0.0      # X轴微分增益
    pid_y_kp: 0.25     # Y轴比例增益
    pid_yaw_kp: 0.5    # 偏航角比例增益

路径跟踪参数:
    position_tolerance_cm: 9.0   # 位置容差 (厘米)
    yaw_tolerance_deg: 5.0       # 角度容差 (度)
    height_tolerance_cm: 12.0    # 高度容差 (厘米)

串口通信参数:
    port: /dev/ttyUSB0
    baudrate: 921600
    update_rate: 20.0 Hz         # 更新频率

坐标系参数:
    map_frame: "map"
    laser_link_frame: "laser_link"
```

---

这个架构图展示了 Fly Car 系统的各个组件如何协同工作，从传感器数据采集到最终的电机控制。
