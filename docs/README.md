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
  - `src/activity_control_pkg/` — 航点发布 `RouteTargetPublisher`（队列+到达推进+z编码起降+`insertNext`插点+`setFlightMode`全局z覆盖+`land_after`落地标志）；并含 `coverage_generator`(弓字形覆盖航点) 与 `obstacle_decision`(墙逼近→原地起飞+飞行模式)，由 `coverage_mission_node` 同进程拼起。详见[地面底盘与飞越](ground_chassis_and_flyover.md)
  - `src/pid_control_pkg/` — 位置 PID，map 系误差→速度，发 `/target_velocity`
  - `src/uart_to_stm32/` — ROS↔STM32 串口桥，下发飞控前做 map→body 旋转
  - `src/my_carto_pkg/` — Cartographer 建图/定位（提供 `map<-laser_link` TF）
  - `src/ground_chassis_pkg/` — **飞车地面差速底盘**(本项目新增,✅代码):`diff_drive_controller`(/target_position+/ground_enable→/cmd_vel) + `chassis_bridge.py`($VW 串口桥,/dev/ttyS6@115200,同 car 的 SR5E1E3) + `chassis_mux`(地空互斥仲裁,按目标 z 与实测 /height 发 /ground_enable、/flight_enable)。地面行驶用它,空中才用飞控链,两者严格互斥。详见[地面底盘与飞越](ground_chassis_and_flyover.md)
  - `src/my_launch/` — 总启动入口
  - `src/serial_comm/` — 串口协议库
  - `src/bluesea2/` — 蓝海激光雷达驱动
  - `src/pose_sender_pkg/` — 飞车位姿 UDP 发送（20Hz 查 `map<-laser_link` TF 发给地面车，供其跟随；协议与对端见 `../car/docs/follow_fly_car_design.md` §2.1。本项目新增，✅。注意 `ROS_DOMAIN_ID` 必须与车板的 10 错开。`target_ip` 默认 `192.168.10.161`＝车板在路由器网的地址，飞车自身 `192.168.10.171`，端口 8888）
  - `src/wifi_sta_manager/` — **⚠ 已作废（2026-06-14 组网改路由器）**：旧自建热点方案的 STA 客户端（连车端 AP，静态 `192.168.50.x`）。现两板均开机 autoconnect 路由器，**直接绕过本包，不再启动**。组网现状见 [wifi_lan_autostart](wifi_lan_autostart.md) §七

**硬约束（必须遵守）**：
- **双设备开发**：本地只写代码 + git；编译运行在开发板上（`colcon build`），代码用 **syncpi** 传过去。**本地不要执行 `colcon build`，也不要声称"已编译验证"**。验证以开发板为准。
- **坐标系**：位姿/误差/检测都在 `map` 系算；`laser_link` 是车体激光系。下发飞控前由 `uart_to_stm32` 用当前 yaw 做 `map→body` 旋转（`Rz(yaw)`）。当前速度和目标速度两条通路都要旋转，详见[速度坐标系修正](velocity_frame_fix_and_workflow.md)。
- **单位**：航点/速度对外多用 cm、cm/s、deg；TF 为 m。注意转换。

**话题速查**：
| 话题 | 类型 | 产生者 → 消费者 | 说明 |
| --- | --- | --- | --- |
| `/scan` | LaserScan | 雷达 → 检测 | 单线激光 |
| `/detected_obstacle` | Float32MultiArray | obstacle_detector → 决策 | 阻挡当前航点路线的折线顶点串，布局见[包文档](obstacle_detector_pkg.md) |
| `/obstacle_detect_enable` | Bool | 决策 → obstacle_detector | 边沿使能检测 |
| `/target_position` | Float32MultiArray | RouteTargetPublisher → pid | `[x_cm,y_cm,z_cm,yaw_deg]` |
| `/target_velocity` | Float32MultiArray | pid → uart_to_stm32 | `[vx,vy,vz cm/s, vyaw deg/s]` |
| `/velocity_map` | Twist | (里程) → uart_to_stm32 | 当前速度，map 系 |
| `/height` | Int16 | uart_to_stm32 → pid/route/mux | 高度 cm |
| `/is_st_ready` `/mission_step` | UInt8 | 飞控 → ROS | 飞控就绪/任务步 |
| `/ground_enable` `/flight_enable` | Bool | chassis_mux → diff_drive/pid | 地空互斥使能(latched),永不同时为真 |
| `/cmd_vel` | Twist | diff_drive → chassis_bridge | 地面差速 v(m/s)/w(rad/s) → `$VW` |
| `/detected_obstacle` | Float32MultiArray | obstacle_detector → obstacle_decision | 阻挡墙顶点串+path_dist |

**接手开发时**：先读[任务架构](coverage_flyover_mission_design.md)了解主线与待实现项 → 再读相关[包文档](#功能包说明) → 动手前确认是否触发了"双设备/坐标系/单位"三条硬约束。

---

## 总览：数据链路

```
传感器/检测                 决策/航点                 控制/下发              飞控
─────────                  ─────────                ─────────             ────
/scan + TF                                                                  ↑
  ├─ obstacle_detector ◄─ /target_position（当前地面航点）
  │         └──────────► /detected_obstacle（路线阻挡墙 + path_dist）─► 障碍决策(待实现) ─┐
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
- [飞车地面底盘 + 起飞/飞越/落地](ground_chassis_and_flyover.md) — 陆地差速底盘与空中飞控**两套控制链如何互斥切换**:chassis_mux 仲裁(目标z起飞、实测/height降落、2s缓冲)、原地垂直起飞、全局z覆盖、land_after落地标志、isReached容忍度方向。**地空切换的实现细节看这篇。**

### 功能包说明
- [obstacle_detector_pkg 折线障碍检测](obstacle_detector_pkg.md) — 激光+TF 的折线墙检测(Split-and-Merge)，话题/数据布局/参数/用法。
- [双机自组局域网 + Wi-Fi 自启动（飞车侧）](wifi_lan_autostart.md) — `wifi_sta_manager` 包 + `scripts/autostart_wifi.sh` 自动择网（扫到车热点连局域网，否则连默认上网 `HUAWEI-GR18QG`）。拓扑/地址/单网卡坑/上板验证全在此。

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
| obstacle_detector_pkg 路线阻挡墙检测 | ✅ 开发板静态场景实测通过：识别右侧+前方连续折线，并输出稳定 `path_dist`；动态接近与非阻挡路线仍待测 |
| 覆盖生成器(弓字形航点) | 🔶 代码完成(2026-06-14,`coverage_generator`);场地边界/行距占位待标定 |
| 障碍决策(墙逼近→原地起飞+飞行模式) | 🔶 代码完成(2026-06-14,`obstacle_decision`,简化为 path_dist≤0.6 触发→插原地起飞点+`setFlightMode`);待上板实测 |
| 飞车地面差速底盘 + 地空互斥切换 | 🔶 代码完成(2026-06-14,`ground_chassis_pkg`:diff_drive_controller/chassis_bridge/chassis_mux + pid 加 /flight_enable 标志位);串口/高度阈值/PID 待上板实测。详见[地面底盘与飞越](ground_chassis_and_flyover.md) |
| 落地(land_after 标志 → 原地垂直下降) | 🔶 代码完成(2026-06-14);落地判据依赖飞控回传 /height |
