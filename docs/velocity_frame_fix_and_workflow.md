# 速度坐标系修正与开发工作流

## 一、本次修改：目标速度坐标系修正

### 背景

ROS 与 STM32 飞控之间通过 `uart_to_stm32` 节点桥接速度数据，存在两条速度下发通路：

- **当前速度** `/velocity_map`（`geometry_msgs/Twist`）
- **目标速度** `/target_velocity`（`std_msgs/Float32MultiArray`，`[vx_cm/s, vy_cm/s, vz_cm/s, vyaw_deg/s]`）

飞控期望接收的速度均为**机体系（body frame）**。

### 问题

两条通路送进飞控时坐标系不一致：

| 通路 | 数据来源坐标系 | 下发前处理 | 实际送入飞控的坐标系 |
| --- | --- | --- | --- |
| 当前速度 `/velocity_map` | map 系 | `transformVelocity(v, current_yaw_)` 旋转 | 机体系 ✅ |
| 目标速度 `/target_velocity` | map 系 | 无旋转，直接下发 | map 系 ❌ |

目标速度由 `pid_control_pkg` 的 `PositionPIDController` 计算：位姿取自 `map -> laser_link` 的 TF，误差 `error_x/error_y` 与速度分量 `vel_x/vel_y` 都在 **map 系**下分解（见 `pid_controller.cpp` 的 `getCurrentPose()` / `processPID()`）。因此 `/target_velocity` 发出的是 map 系速度，而非机体系。

后果：目标速度方向会随机头朝向（yaw）整体偏转，控制方向错误。

### 修改内容

文件：`src/uart_to_stm32/src/uart_to_stm32.cpp`，函数 `targetVelocityCallback`。

在下发前对目标速度做与当前速度通路一致的 map→body 旋转，复用已有的 `transformVelocity(v, current_yaw_)`：

```cpp
if (yaw_valid_) {
  const Eigen::Vector3d v_map(vx_cm_per_s, vy_cm_per_s, vz_cm_per_s);
  const Eigen::Vector3d v_body = transformVelocity(v_map, current_yaw_);
  sendTargetVelocityToSerial(v_body.x(), v_body.y(), v_body.z(), vyaw_deg_per_s);
} else {
  // 尚未取得 yaw，退回原样发送并告警
  sendTargetVelocityToSerial(vx_cm_per_s, vy_cm_per_s, vz_cm_per_s, vyaw_deg_per_s);
}
```

旋转矩阵 `Rz(yaw)`（map→body）：

```
Rz = [  cos(yaw)  sin(yaw)  0
       -sin(yaw)  cos(yaw)  0
        0         0         1 ]
```

### 要点

- **vz 不受影响**：`Rz` 第三行为 `[0,0,1]`，z 速度分量旋转前后不变。
- **vyaw 不受影响**：角速度不参与坐标旋转，原样下发。
- **单位/缩放不变**：旋转只改方向；`sendTargetVelocityToSerial` 仍按 cm/s 取整打包，逻辑未动。
- **yaw_valid_ 保护**：TF 尚未就绪时退回原样发送并告警，避免用未初始化的 yaw（0）做错误旋转。

### 验证

本地未执行 `colcon build`（开发机与开发板系统版本不同，编译在开发板上进行）。修改为源码级，构建/上板验证在开发板侧完成。

---

## 二、开发工作流（双设备）

本项目采用**双设备开发**模式：

- **本地开发机**：负责代码编写、阅读、静态检查，以及 **git 版本迭代**（commit / branch / push 到 `origin`，远程 `git@github.com:GGbond-dot/fly_car.git`）。本地**不执行** `colcon build`，因为开发机与开发板系统版本不同（开发板为 ROS 2 Humble）。
- **开发板（飞控载体）**：负责 `colcon build` 编译、运行与上板实测。

### 代码传输：syncpi

本地与开发板之间的代码同步使用 **syncpi** 完成（本地 → 开发板传代码）。

### 典型循环

1. 本地编写/修改代码。
2. 本地做 git 版本迭代（在功能分支上 commit，必要时 push）。
3. 用 syncpi 把代码传到开发板。
4. 在开发板上 `colcon build` 编译、运行、实测。
5. 根据实测结果回到本地继续迭代。

> 注意：因编译在开发板侧进行，本地的"完成"仅代表源码级修改完成，真正的构建与功能验证以开发板为准。
