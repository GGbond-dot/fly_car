# Fly Car 🚗

这个是我的第一个开源 project - 基于 ROS2 的智能自主小车系统

## 项目简介

Fly Car 是一个集成了多种传感器和控制算法的自主智能小车项目，实现了：
- ✅ 自主导航和路径跟踪
- ✅ PID 位置控制
- ✅ 激光雷达 SLAM 建图
- ✅ 蓝牙和串口通信
- ✅ 视觉识别（OpenMV）

## 📚 学习资源

### 期末考复习材料

- **[学习指南 (STUDY_GUIDE.md)](./STUDY_GUIDE.md)** - 详细的技术知识点总结和考试重点
- **[系统架构 (ARCHITECTURE.md)](./ARCHITECTURE.md)** - 系统架构图和数据流向图
- **[ROS2 命令速查表 (ROS2_COMMANDS.md)](./ROS2_COMMANDS.md)** - 常用 ROS2 命令快速参考

这些文档涵盖了：
- ROS2 核心概念和通信机制
- PID 控制理论和调参方法
- 坐标变换和 SLAM 原理
- 系统架构和模块设计
- 常见问题和调试技巧
- ROS2 实用命令和调试方法

## 快速开始

### 编译项目
```bash
cd /path/to/workspace
colcon build
source install/setup.bash
```

### 运行系统
```bash
# 启动完整系统
ros2 launch uart_to_stm32 total.launch.py
```

## 主要模块

| 模块 | 功能 |
|------|------|
| `pid_control_pkg` | PID 位置控制器 |
| `activity_control_pkg` | 路径规划和目标发布 |
| `uart_to_stm32` | STM32 串口通信 |
| `bluetooth` | 蓝牙遥控 |
| `bluesea2` | 激光雷达驱动 |
| `my_carto_pkg` | Cartographer SLAM |
| `openmv_bridge` | OpenMV 视觉桥接 |
| `car_driver` | 车辆底层驱动 |

## 技术栈

- **ROS2** - 机器人操作系统
- **C++17** - 主要编程语言
- **Python** - 启动脚本
- **Cartographer** - SLAM 算法
- **TF2** - 坐标变换

## 贡献

欢迎提交 Issue 和 Pull Request！

## 许可证

Apache-2.0
