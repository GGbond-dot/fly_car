# 平地遍历 + 遇障起飞 —— 任务架构设计

> 状态：架构已定，等场地尺寸到位后填参数并实现"覆盖生成器"和"障碍决策"两个节点。
>
> **§八 已于 2026-07-14 晚被取代**：飞车陆地段与车搜索区的下发方式已改，
> 见 [`terminal/project_markdown/patrol_mission_planning_design.md`](../../terminal/project_markdown/patrol_mission_planning_design.md)。
> §一~§七 的地空 z 编码、障碍决策、`chassis_mux` 仲裁仍然有效。

## 一、目标

车在平地上**系统遍历已知场地**（弓字形扫一遍），途中前方有一道**很长的折线墙**（约 3~4 段），无法绕行、只能飞越。要求：遍历过程中检测到墙逼近时，自动起飞 → 越过 → 对面降落 → 继续遍历。

场景约束（已确认）：

- 遍历类型：**已知场地系统扫一遍**（非未知探索、非固定路线）。
- 地面障碍：**只有这一道要飞越的墙**，没有其他需要绕行的障碍。
- 传感器：单线激光雷达（`/scan`）+ TF（`map <- laser_link`）。

## 二、关键结论：不需要任何路径规划器

- ❌ **EGO-Planner**：是无人机的 3D 轨迹规划器，需要深度相机 / 3D 点云，且用于三维杂乱环境穿梭避障。本任务是"过一道墙"，且只有单线激光，**不适用**。
- ❌ **Nav2**：地面 2D 导航栈，用于"绕开多个地面障碍做路由"。本任务地面只有这一道墙、且是飞越不是绕行，**没有绕行需求，不需要**。
- ✅ **弓字形覆盖航点 + 现有航点跟踪 + 障碍触发插入越障航点**。任务本质是"按顺序去一串点"，现有件已经在做。

判据：是否需要规划器，取决于"有没有要动态绕行的障碍"。本任务没有 → 不需要。

## 三、复用现有件（重要）

`activity_control_pkg/RouteTargetPublisher` 已经实现了任务所需的核心机制：

- 维护一个航点队列 `Target{x_cm, y_cm, z_cm, yaw_deg}`，发布当前航点到 `/target_position`；
- 用 TF 监测到达（`isReached`），到达后自动推进下一个；
- **起飞/降落直接用航点 z 值编码**：`isReached` 中 z > 20cm 视为空中航点（放宽 xy/yaw 容差），z 低视为地面/降落航点。

→ "地面跑(z 低) → 飞(z 高) → 落(z 低)" 完全可以只用航点 z 表达，无需额外起降状态机。

下游链路：`/target_position` →（**2026-06-14 起经 `chassis_mux` 按 z 仲裁**）→ 地面态走 `diff_drive_controller`→`chassis_bridge`→差速底盘；空中态走 `pid_control_pkg`→`uart_to_stm32`→飞控。两条链严格互斥，详见[地面底盘与飞越](ground_chassis_and_flyover.md)。

## 四、目标架构

```
覆盖生成器──弓字形地面航点(z=巡航低值)────────┐
                                              ├─► RouteTargetPublisher ─/target_position─► chassis_mux ─┬─(z低/已落地)─► diff_drive ─► 地面差速底盘
障碍决策──墙近→原地起飞点+全局z覆盖(setFlightMode)┘                                                      └─(z高/在空中)─► pid_control ─► uart_to_stm32 ─► 飞控
                              ▲
        obstacle_detector(静态场景实测通过)──/detected_obstacle(路线阻挡墙 + path_dist)
```

### 模块清单

| 模块 | 状态 | 职责 |
| --- | --- | --- |
| `obstacle_detector_pkg` | ✅ 静态场景上板实测通过 | 激光+TF 拟合所有有效墙体，结合当前地面 `/target_position` 只发布阻挡路线的最近墙及 `path_dist` |
| `RouteTargetPublisher` | ✅ 已存在 | 航点队列 + 到达推进 + z 编码起降 |
| `pid_control_pkg` / `uart_to_stm32` | ✅ 已存在 | 航点跟踪 + 下发飞控 |
| **覆盖生成器** | 🔶 代码完成 | 已知场地边界 + 行距 → 生成弓字形地面航点灌入队列（`coverage_generator`，场地参数占位待标定） |
| **障碍决策** | 🔶 代码完成 | 订阅 `/detected_obstacle`，`path_dist ≤ 0.6m` 时插原地起飞点 + `setFlightMode`（见上 §五-2 简化说明） |
| **地面差速底盘 + 地空互斥** | 🔶 代码完成 | `ground_chassis_pkg`：飞车地面行驶用差速底盘，与飞控按目标 z/实测 height 互斥切换。详见[专文](ground_chassis_and_flyover.md) |

## 五、两个待实现节点的设计

### 1. 覆盖生成器（`coverage_generator`，2026-07-14 起双模式）

`mode` 参数切换，详见 §八（地面站选区驱动）。
- **`l_path`（飞车任务默认）**：L 形，只转 1 个直角弯。飞车地面没调好、不宜多走 —— 起点→拐角（障碍前）→转 90°→终点，两航点用 `l_start/l_corner/l_end` 参数（map 系 cm）。走完起飞飞越归障碍决策/z 编码，不在本生成器。
- **`boustrophedon`（单机自测/整片覆盖）**：弓字形，贴 1m 网格来回扫（一行正向 yaw=0、下一行反向 yaw=180，全部 z=`cruise_z`，`addTarget()`）。区域边界吸附网格线、行距=`grid_cell_cm`×`lane_cells`。

### 2. 障碍决策（遇墙起飞）

> **2026-06-14 简化（用户思路）**：不再算墙几何、不插"起飞/越过/落点"三点。xy 航点开跑前全发布、z 只是模式指示；遇墙只需"原地垂直起飞 + 全局 z 覆盖沿原 xy 飞越"。完整设计见[飞车地面底盘 + 起飞/飞越/落地](ground_chassis_and_flyover.md)，此处只留要点：

- 订阅 `/detected_obstacle` 的 `path_dist`。
- 触发：`path_dist ≤ approach_threshold_m`（默认 0.6m，以车为心半径）。
- 动作两步：
  1. `RouteTargetPublisher::insertNext([当前 xy, z=flyover_z])` —— 插当前位置高 z 点 → **原地垂直起飞**。
  2. `RouteTargetPublisher::setFlightMode(true, flyover_z)` —— **全局 z 覆盖**，后续原 xy 航点都在 `flyover_z` 高度飞越（xy 不变）。
- 落地：航点 `land_after` 标志位触发（不在本节点）；越障/落地与地面差速底盘的互斥切换由 `chassis_mux` 负责。详见专文。

## 六、待定参数（场地到位后填）

- **场地边界**：`x_min/x_max/y_min/y_max`（map 系，cm 或 m）
- **行距** `lane_spacing`
- **巡航高度** `cruise_z`（地面值，参考现有测试 ~4cm）
- **障碍检测 ROI**：固定场地图测试使用 x∈[0,5]m、y∈[-4,0]m
- **路线走廊半宽** `path_corridor_half_width_m`（当前默认 0.30m，需按车体宽度与定位误差实测）
- **逼近触发阈值** `approach_threshold`（车到墙多近开始起飞）
- **越障安全高度** `flyover_z`
- **墙前后安全余量**（起飞点距墙、落点距墙各留多少）

## 七、备注

- 折线检测假设墙连续；若实测拐点很尖锐或断开，调 `split_threshold_m` / `chain_break_dist_m`。
- 按双设备工作流：本地写代码 + git，开发板 `colcon build` 与实测，syncpi 传输。

## 八、地面站选区驱动的车+飞车地面覆盖（2026-07-14）

> ⚠️ **本节已被取代（2026-07-14 晚）**，保留仅为记录旧做法，**不要照此实现**。
> 现行设计见终端侧 [`patrol_mission_planning_design.md`](../../terminal/project_markdown/patrol_mission_planning_design.md)。
>
> 两处关键变更：
> 1. **飞车的陆地段不再由飞车端 `coverage_generator(mode=l_path)` 自己生成**。改由终端 planner
>    按实测障碍折线算出转场路线，当 `z=0` 航点混在 `/wildlife/waypoints` 里走 **FC0A** 下发。
>    理由：只有终端手里有地图和实测障碍，飞车端拍 `l_start/l_corner/l_end` 参数是瞎猜。
>    因此 **FC09 不再触发飞车 L 形**（终端侧 `dispatch(start_flycar=False)`），FC09 目前空出未用。
> 2. **车的搜索区不再靠手动框选**。分界线由 FC03 实测障碍决定，左侧整块自动派给车。
>    网格也从 1m 粗格改为 **0.5m（10×8）**。

用户思路定案：地面站在地图上**框选一块遍历区**，一点即下发；**车做主力弓字形覆盖**（多转弯无所谓），**飞车只走极简 L 形**（1 个直角弯，飞车地面没调好、不宜多走），**不平均分**（框选区只给车，飞车 L 独立参数定义）。路径**贴 web 地图 1m 网格**走。

### 全链路

```
地面站(/slam 页,car 上跑) 框选矩形(贴1m网格,coverage_select.js)
   → POST /api/coverage_region {minX,maxX,minY,maxY}(米)
   → kian_ai coverage_dispatch_bridge:
        ├─ 车: 本地发 /coverage_area(Float32MultiArray[x_min,x_max,y_min,y_max]cm, latched)
        │        → follower_pkg/coverage_route_publisher: 贴格子弓字形航点 → /target_position → diff_drive
        └─ 飞车: 发 /flycar_coverage_start(Bool)
                 → 车 xmachine_bridge → UDP FC09 → 飞车 xmachine_bridge → 本地发 /coverage_area
                 → activity_control_pkg/coverage_generator(mode=l_path): L 形航点(用 l_* 参数) → RouteTargetPublisher
```

### 地图坐标系（/slam 页）

固定 5m×4m 俯视图，**ROS map 系：x∈[0,5]m 向右、y∈[0,4]m 向上、原点左下**。SVG `map-grid` = 1m/格（框选吸附的网格）。obstacle-overlay viewBox 500×400(cm)，`toSvgPoint` y 翻转。coverage_select.js 屏幕→米→吸附 1m→POST。

### 参数（尺寸没量，先占位，现场量了填）

- 飞车 L：`l_start_x/y_cm`(固定起点)、`l_corner_x/y_cm`(障碍前拐角)、`l_end_x/y_cm`(转弯后终点)。map 系 cm。
- 车/飞车弓字形：`grid_cell_cm`(默认100=1m,与 web 网格一致)、`lane_cells`(行距=格子×几,默认1)。
- 车到达容差 `pos_tol_cm/yaw_tol_deg`(≥diff_drive)。

### 跨机协议

新增 `FC09`(车→飞车,BoolPacket,burst 重发)：地面站选区后"覆盖开跑"。飞车侧收到本地发 `/coverage_area`(latched, [0,0,0,0] 占位,l_path 只当开跑信号)。见 relief_drop_mission_design.md §三。

### 现场待调

- 飞车 L 形拐角/终点参数按场地量；生成器会警告拐角偏离 90°。
- 车弓字形每行末 180° 掉头接近原地转,车前驱+后万向轮不擅长 —— 可能要把行末做成小圆弧过渡(现只出标准弓字形航点)。
- 飞车走完 L 的"起飞→飞越→覆盖另一边"仍走原障碍决策/z 编码,尚未与 L 终点自动衔接(TODO)。
