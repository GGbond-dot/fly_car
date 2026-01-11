# Fly Car 项目期末考复习指南

## 项目概述

Fly Car 是一个基于 ROS2 的自主智能小车项目，集成了多种传感器和控制算法，实现了自主导航、路径跟踪和远程控制等功能。

## 核心技术栈

- **ROS2 (Robot Operating System 2)**: 机器人操作系统框架
- **C++17**: 主要编程语言
- **Python**: 用于启动脚本和部分节点
- **CMake**: 构建系统
- **TF2**: 坐标变换库

## 系统架构

### 主要功能模块

#### 1. PID 控制模块 (`pid_control_pkg`)
**功能**: 实现位置和姿态的 PID 控制器

**关键概念**:
- **PID控制器**: 比例-积分-微分控制器
  - **P (比例)**: 根据误差的当前值产生控制输出
  - **I (积分)**: 根据误差的累积值消除稳态误差
  - **D (微分)**: 根据误差的变化率提供阻尼，减少超调

**应用场景**: 
- X 轴位置控制
- Y 轴位置控制
- 偏航角（Yaw）控制

**参数配置**:
```
pid_x_kp: 0.3    # X轴比例增益
pid_y_kp: 0.25   # Y轴比例增益
pid_yaw_kp: 0.5  # 偏航角比例增益
```

#### 2. 活动控制模块 (`activity_control_pkg`)
**功能**: 路径规划和目标点发布

**核心类**: `RouteTargetPublisherNode`

**主要功能**:
- 发布路径目标点
- 监测当前位置与目标的距离
- 管理路径点序列
- 高度控制（如果支持）

**容差参数**:
- `position_tolerance_cm`: 位置容差（默认 9cm）
- `yaw_tolerance_deg`: 偏航角容差（默认 5°）
- `height_tolerance_cm`: 高度容差（默认 12cm）

#### 3. 串口通信模块 (`uart_to_stm32`)
**功能**: 与 STM32 微控制器进行串口通信

**通信参数**:
- 端口: `/dev/ttyUSB0`
- 波特率: 921600

**数据流**:
- **接收**: STM32 传感器数据（高度、状态等）
- **发送**: 速度控制命令、任务步骤

**关键订阅话题**:
- `/velocity_map`: 速度信息
- `/target_velocity`: 目标速度
- `/bluetooth_data`: 蓝牙数据

**关键发布话题**:
- `/height`: 高度数据
- `/is_st_ready`: STM32 就绪状态
- `/mission_step`: 任务步骤

#### 4. 蓝牙通信模块 (`bluetooth`)
**功能**: 处理蓝牙遥控指令

**依赖**:
- `serial_comm`: 串口通信库
- `sensor_msgs`: 传感器消息类型

#### 5. OpenMV 桥接模块 (`openmv_bridge`)
**功能**: 与 OpenMV 视觉模块通信

#### 6. 激光雷达模块 (`bluesea2`)
**功能**: 处理蓝海 2D 激光雷达数据

**支持的通信方式**:
- UDP 通信
- TCP 通信
- UART 串口通信

#### 7. Cartographer 地图构建 (`my_carto_pkg`)
**功能**: 使用 Google Cartographer 进行 SLAM（同步定位与地图构建）

#### 8. 车辆驱动模块 (`car_driver`)
**功能**: 底层车辆控制接口

#### 9. 启动配置模块 (`my_launch`)
**功能**: 包含各种启动文件，用于快速启动不同的功能组合

## 核心算法详解

### PID 控制算法

PID 控制器的数学表达式：

```
u(t) = Kp * e(t) + Ki * ∫[0→t]e(τ)dτ + Kd * de(t)/dt
```

其中：
- `u(t)`: 控制输出
- `e(t)`: 误差（目标值 - 当前值）
- `Kp`: 比例增益
- `Ki`: 积分增益
- `Kd`: 微分增益

**调参技巧**:
1. 先调 Kp，使系统有基本响应
2. 增加 Ki 消除稳态误差
3. 增加 Kd 减少振荡和超调
4. 设置最大输出限制防止饱和

### TF2 坐标变换

**坐标系关系**:
- `map`: 全局地图坐标系
- `laser_link`: 激光雷达坐标系
- 其他可能的坐标系：`base_link`（机器人基座）、`odom`（里程计）

**变换查询**:
```cpp
try {
  geometry_msgs::msg::TransformStamped transform;
  transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
} catch (const tf2::TransformException & ex) {
  RCLCPP_ERROR(logger, "TF lookup failed: %s", ex.what());
}
```

### 路径跟踪算法

1. **目标点发布**: 按序列发布路径点
2. **距离计算**: 计算当前位置与目标点的欧氏距离
3. **容差判断**: 判断是否到达目标点
4. **下一目标**: 到达后切换到下一个目标点

## ROS2 核心概念

### 节点 (Node)
- 每个功能模块是一个独立的节点
- 节点之间通过话题、服务、动作进行通信

### 话题 (Topic)
- 发布-订阅模式
- 异步通信
- 多对多关系

### 消息类型
- `geometry_msgs::msg::Twist`: 速度消息（线速度和角速度）
- `std_msgs::msg::Float32MultiArray`: 浮点数组
- `std_msgs::msg::UInt8`: 无符号 8 位整数
- `std_msgs::msg::Int16`: 有符号 16 位整数

### QoS (Quality of Service)
- `transient_local()`: 对于晚加入的订阅者，发布最后一条消息
- `reliable()`: 可靠传输
- `KeepLast(n)`: 保留最后 n 条消息

## 系统工作流程

### 典型运行流程

1. **初始化阶段**:
   - 启动 ROS2 节点
   - 建立串口连接
   - 初始化 TF 监听器
   - 配置 PID 参数

2. **定位阶段**:
   - 激光雷达扫描环境
   - Cartographer 构建/加载地图
   - TF 系统维护坐标变换

3. **控制阶段**:
   - 接收目标位置
   - PID 计算控制输出
   - 发送速度命令到 STM32
   - STM32 控制电机

4. **反馈阶段**:
   - 读取传感器数据
   - 更新当前状态
   - 计算误差
   - 调整控制输出

## 重要数据结构

### Float32MultiArray 格式
在本项目中常用于传递位置和姿态信息：
```
data[0]: X 位置 (cm)
data[1]: Y 位置 (cm)
data[2]: 偏航角 (degrees)
```

### Twist 消息
```cpp
geometry_msgs::msg::Twist {
  linear.x: 前进速度 (m/s)
  linear.y: 横向速度 (m/s)
  linear.z: 垂直速度 (m/s)
  angular.x: 滚转角速度 (rad/s)
  angular.y: 俯仰角速度 (rad/s)
  angular.z: 偏航角速度 (rad/s)
}
```

## 常见问题和调试技巧

### 1. 串口连接失败
- 检查设备路径：`ls -l /dev/ttyUSB*`
- 检查权限：`sudo chmod 666 /dev/ttyUSB0`
- 检查波特率是否匹配

### 2. TF 变换查询失败
- 使用 `ros2 run tf2_tools view_frames` 查看 TF 树
- 使用 `ros2 topic echo /tf` 查看变换信息
- 检查坐标系名称是否正确

### 3. PID 调参不佳
- 系统振荡：降低 Kp 或增加 Kd
- 响应太慢：增加 Kp
- 稳态误差：增加 Ki
- 使用 `rqt_plot` 实时查看误差曲线

### 4. 节点通信问题
- 使用 `ros2 node list` 查看活动节点
- 使用 `ros2 topic list` 查看所有话题
- 使用 `ros2 topic echo <topic_name>` 监听话题数据
- 使用 `ros2 topic hz <topic_name>` 检查发布频率

## 编译和运行

### 编译项目
```bash
cd /path/to/workspace
colcon build
source install/setup.bash
```

### 运行示例
```bash
# 启动完整系统
ros2 launch uart_to_stm32 total.launch.py

# 启动 PID 控制器
ros2 launch pid_control_pkg position_pid_controller.launch.py

# 启动路径跟踪
ros2 launch activity_control_pkg route_target_publisher.launch.py

# 启动地图构建
ros2 launch my_carto_pkg fly_carto.launch.py
```

## 关键知识点总结

### 控制理论
1. **PID 控制器原理**和参数调整
2. **死区 (Dead Zone)** 的作用：防止小误差引起频繁控制
3. **输出限幅**：防止控制量过大

### 机器人学
1. **坐标系变换**：理解不同坐标系之间的关系
2. **位姿表示**：(x, y, yaw) 或 (x, y, z, roll, pitch, yaw)
3. **SLAM 原理**：同步定位与地图构建

### ROS2 架构
1. **节点通信机制**：话题、服务、动作
2. **消息定义**：标准消息类型和自定义消息
3. **生命周期管理**：节点的启动、配置、激活等状态
4. **QoS 策略**：消息传输的可靠性和持久性

### 软件工程
1. **模块化设计**：每个功能独立封装
2. **接口设计**：通过话题定义清晰的接口
3. **错误处理**：日志记录和异常处理
4. **参数配置**：使用 ROS 参数系统实现灵活配置

## 考试重点提示

### 理论部分
- PID 控制器的数学原理和调参方法
- ROS2 的通信机制（话题、服务、动作）
- 坐标变换的概念和应用
- SLAM 的基本原理

### 实践部分
- 如何编写 ROS2 节点
- 如何使用 TF2 进行坐标变换
- 如何配置和调试 PID 参数
- 串口通信的实现方法

### 系统设计
- 如何设计模块化的机器人系统
- 话题命名和消息类型的选择
- 错误处理和日志记录的最佳实践

## 扩展学习资源

1. **ROS2 官方文档**: https://docs.ros.org/
2. **PID 控制理论**: 现代控制理论教材
3. **TF2 教程**: ROS2 官方 TF2 教程
4. **Cartographer 文档**: Google Cartographer 官方文档

---

*祝考试顺利！Good luck with your exams!*
