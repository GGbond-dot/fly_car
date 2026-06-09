# fly_car 开发文档索引

本目录记录 fly_car 项目的设计决策、功能包说明与开发记录。新增文档请同步更新本索引。

> 本文件同时作为**新对话/AI 接手开发的入口**。导入新会话时，先读下面的"给 AI 的上下文"再读正文。

---

## 给 AI 的上下文（冷启动必读）

**这是什么项目**：`fly_car` 是一个 ROS 2（开发板为 Humble）的"陆空两用车"项目——车在平地行驶，遇到无法绕行的长障碍物（折线墙）时起飞越过。核心目标见[任务架构](coverage_flyover_mission_design.md)。

**仓库位置 / 结构**：
- 工作区根：`fly_car/`（`src/` 下为各功能包，`docs/` 为本文档目录）。
- 关键功能包（路径均相对工作区根）：
  - `src/obstacle_detector_pkg/` — 折线障碍检测（本项目新增，✅）
  - `src/activity_control_pkg/` — 航点发布 `RouteTargetPublisher`（队列+到达推进+z编码起降）
  - `src/pid_control_pkg/` — 位置 PID，map 系误差→速度，发 `/target_velocity`
  - `src/uart_to_stm32/` — ROS↔STM32 串口桥，下发飞控前做 map→body 旋转
  - `src/my_carto_pkg/` — Cartographer 建图/定位（提供 `map<-laser_link` TF）
  - `src/my_launch/` — 总启动入口
  - `src/serial_comm/` — 串口协议库
  - `src/bluesea2/` — 蓝海激光雷达驱动

**硬约束（必须遵守）**：
- **双设备开发**：本地只写代码 + git；编译运行在开发板上（`colcon build`），代码用 **syncpi** 传过去。**本地不要执行 `colcon build`，也不要声称"已编译验证"**。验证以开发板为准。
- **坐标系**：位姿/误差/检测都在 `map` 系算；`laser_link` 是车体激光系。下发飞控前由 `uart_to_stm32` 用当前 yaw 做 `map→body` 旋转（`Rz(yaw)`）。当前速度和目标速度两条通路都要旋转，详见[速度坐标系修正](velocity_frame_fix_and_workflow.md)。
- **单位**：航点/速度对外多用 cm、cm/s、deg；TF 为 m。注意转换。

**话题速查**：
| 话题 | 类型 | 产生者 → 消费者 | 说明 |
| --- | --- | --- | --- |
| `/scan` | LaserScan | 雷达 → 检测 | 单线激光 |
| `/detected_obstacle` | Float32MultiArray | obstacle_detector → 决策 | 折线顶点串，布局见[包文档](obstacle_detector_pkg.md) |
| `/obstacle_detect_enable` | Bool | 决策 → obstacle_detector | 边沿使能检测 |
| `/target_position` | Float32MultiArray | RouteTargetPublisher → pid | `[x_cm,y_cm,z_cm,yaw_deg]` |
| `/target_velocity` | Float32MultiArray | pid → uart_to_stm32 | `[vx,vy,vz cm/s, vyaw deg/s]` |
| `/velocity_map` | Twist | (里程) → uart_to_stm32 | 当前速度，map 系 |
| `/height` | Int16 | uart_to_stm32 → pid/route | 高度 cm |
| `/is_st_ready` `/mission_step` | UInt8 | 飞控 → ROS | 飞控就绪/任务步 |

**接手开发时**：先读[任务架构](coverage_flyover_mission_design.md)了解主线与待实现项 → 再读相关[包文档](#功能包说明) → 动手前确认是否触发了"双设备/坐标系/单位"三条硬约束。

---

## 总览：数据链路

```
传感器/检测                 决策/航点                 控制/下发              飞控
─────────                  ─────────                ─────────             ────
/scan + TF                                                                  ↑
  ├─ obstacle_detector ─► /detected_obstacle ─► 障碍决策(待实现) ─┐
  │                                                               ├─► RouteTargetPublisher
  └─ (覆盖生成器, 待实现) ──── 弓字形地面航点 ───────────────────┘   │ /target_position
                                                                       ▼
                                                          pid_control_pkg(map系误差→速度)
                                                                       │ /target_velocity, /velocity_map
                                                                       ▼
                                                          uart_to_stm32(map→body旋转→串口) ─► STM32 飞控
```

## 文档分类

### 架构 / 任务设计
- [平地遍历 + 遇障起飞 任务架构](coverage_flyover_mission_design.md) — 整体架构、为何不用规划器(EGO/Nav2)、复用现有航点机制、待实现节点与待定参数。**后续开发主线，先读这篇。**

### 功能包说明
- [obstacle_detector_pkg 折线障碍检测](obstacle_detector_pkg.md) — 激光+TF 的折线墙检测(Split-and-Merge)，话题/数据布局/参数/用法。

### 修改 / 开发记录
- [速度坐标系修正与开发工作流](velocity_frame_fix_and_workflow.md) — uart_to_stm32 目标速度补 map→body 旋转的修正记录；附双设备开发工作流。
- [基本航点飞行裁剪开发记录](basic_waypoint_flight_prune.md) — basic-waypoint-flight-prune 分支裁剪记录(保留/删除范围)。

## 关键约定速查

- **坐标系**：`map`(世界) / `laser_link`(车体激光)。位姿与误差在 map 系算；下发飞控前由 uart_to_stm32 旋转到机体系。详见[速度坐标系修正](velocity_frame_fix_and_workflow.md)。
- **航点 z 编码起降**：`RouteTargetPublisher` 中 z>20cm 视为空中航点(放宽 xy/yaw)，z 低视为地面/降落。详见[任务架构](coverage_flyover_mission_design.md)。
- **开发工作流**：本地写代码+git，开发板 `colcon build` 与实测，syncpi 传输；本地不编译。详见[工作流](velocity_frame_fix_and_workflow.md#二开发工作流双设备)。

## 当前进度

| 模块 | 状态 |
| --- | --- |
| obstacle_detector_pkg 折线检测 | ✅ 已完成(本地，待上板编译) |
| 覆盖生成器(弓字形航点) | ⬜ 待实现(等场地参数) |
| 障碍决策(遇墙插入越障航点) | ⬜ 待实现(等场地参数) |
