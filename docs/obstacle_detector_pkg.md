# 长障碍物（墙）激光检测 obstacle_detector_pkg

## 背景与目标

车在平地上行驶，前方有一道**很长、无法绕行**的障碍物（墙），只能飞越。本包用激光雷达检测这道墙的位置，为后续"在墙前何处起飞/何处降落"的决策提供几何信息。

实现借鉴 kian_26fly 的 `pillar_detector_pkg`（TF + 激光），但目标几何不同：

- 柱子检测：离散小目标，做"邻近分组聚成点"，输出多个点。
- 墙检测：一道连续的**折线**障碍（3~4 段），做 **Split-and-Merge 折线拟合**，输出有序顶点串。

## 算法

由 `/obstacle_detect_enable`（`std_msgs/Bool`，transient_local）边沿使能。使能期间，每帧 `/scan`：

1. 用 TF `map <- laser_link`（查不到当前帧时间戳则回退最新）把每个激光点变换到 **map 系**（保持扫描序），同时取出车（laser 原点）在 map 系的位置。
2. 用 ROI bbox（`roi_x/y_min/max_m`，map 系）过滤出墙所在区域的点。
3. 按相邻点距离 `chain_break_dist_m` 断链，分别拟合所有有效墙体。
4. **Split（Douglas–Peucker）**：从首尾连线开始，在残差最大且 > `split_threshold_m` 处递归劈开，得到顶点。
5. **Merge**：相邻两段转角 < `merge_collinear_deg` 视为近共线，删掉中间顶点去碎段。
6. 链点数 < `min_chain_points` 或总长 < `min_total_length_m` 判为无效，不发布。
7. 订阅当前地面航点 `/target_position`，选择与“当前位置→当前航点”路径走廊相交且沿路线最近的墙。
8. 仅将该路线阻挡墙连续发布到 `/detected_obstacle`；空中航点不执行选择。

## 话题接口

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | 订阅 | 激光雷达 |
| `/obstacle_detect_enable` | `std_msgs/Bool` | 订阅 | 边沿使能，true 期间逐帧检测 |
| `/target_position` | `std_msgs/Float32MultiArray` | 订阅 | 当前航点 `[x_cm,y_cm,z_cm,yaw_deg]`，仅 z≤20cm 的地面航点参与检测 |
| `/detected_obstacle` | `std_msgs/Float32MultiArray` | 发布 | 折线墙顶点串（见下） |
| `/obstacle_debug_points` | `std_msgs/Float32MultiArray` | 发布 | 调参旁路：落进 ROI 的 map 系原始点 [x0,y0,x1,y1,...] |

### `/detected_obstacle` 数据布局（变长，map 系，单位 m）

| 索引 | 含义 |
| --- | --- |
| 0 | N：顶点数（N 个顶点 = N-1 段）|
| 1 .. 2N | 顶点串：x0,y0, x1,y1, …, x(N-1),y(N-1)（有序，首尾相连）|
| 2N+1 | path_dist：沿“当前位置→当前航点”路线到阻挡墙的距离 |
| 2N+2 | total_length：折线总长 |

> 只发布阻挡当前地面航点路线的最近墙。`path_dist` 可供后续起飞决策直接判断接近阈值。

## 参数（见 `launch/obstacle_detector.launch.py`）

- `scan_topic` `/scan`，`enable_topic` `/obstacle_detect_enable`
- `map_frame` `map`，`laser_link_frame` `laser_link`
- `roi_x_min_m / roi_x_max_m / roi_y_min_m / roi_y_max_m`：墙所在世界区域，**必须按场地标定**（默认 x∈[0.2,6.0]、y∈[-3.0,3.0] 仅为占位）
- `chain_break_dist_m`(0.20)：相邻点超此距离断链
- `min_chain_points`(15)：点链最少点数
- `split_threshold_m`(0.05)：点到段超此距离则劈开（越小段越多越贴合）
- `merge_collinear_deg`(10.0)：相邻段转角小于此值则合并（去碎段）
- `min_total_length_m`(0.50)：折线总长下限
- `path_corridor_half_width_m`(0.30)：当前路线走廊半宽；距离路线超过此值的侧墙不会被选中
- `tf_timeout_sec`(0.05)
- `publish_debug_points`(true) / `debug_points_topic`

## 用法

固定场地图测试（同时启动 Cartographer 与检测节点，ROI 为 x∈[0,5]、y∈[-4,0]）：

```bash
ros2 launch obstacle_detector_pkg obstacle_detector_test.launch.py
```

或只启动检测节点：

```bash
ros2 launch obstacle_detector_pkg obstacle_detector.launch.py
# 发布一个地面测试航点（cm）；例如从原点沿 +x 检查到 x=300cm 的路线
ros2 topic pub --once --qos-durability transient_local /target_position std_msgs/Float32MultiArray "{data: [300.0, 0.0, 4.0, 0.0]}"
# 开始检测
ros2 topic pub --once /obstacle_detect_enable std_msgs/Bool "{data: true}"
# 查看结果
ros2 topic echo /detected_obstacle
# 停止
ros2 topic pub --once /obstacle_detect_enable std_msgs/Bool "{data: false}"
```

## 开发板实测记录（2026-06-12）

固定起点静态测试中，场景同时存在右侧墙和前方墙。发布穿过前方墙的地面航点后：

- 检测结果稳定为 3 个顶点、2 段连续折线，成功表达右侧墙与前方墙连接形成的墙体。
- `path_dist` 稳定约为 1.72～1.73 m，表示沿当前航点路线到阻挡墙的距离。
- `total_length` 稳定约为 1.99～2.00 m。
- 当前静态场景结果符合预期；后续仍需验证车辆向墙移动时 `path_dist` 连续减小、路线止于墙前时不发布、非阻挡侧墙不误触发。

## 待标定 / 后续

- **ROI 必须按实际场地与起飞点坐标标定**，否则会把无关点也拉进拟合。
- 当前固定场地图测试 ROI 为 x∈[0,5]、y∈[-4,0]，由
  `obstacle_detector_test.launch.py` 传给检测节点。
- 整体目标是"平地遍历地图 → 遇障起飞"：检测节点持续输出折线墙，由上层任务节点用 `path_dist` 判断"沿当前路线逼近到墙前阈值"→ 触发起飞越障。
- 跨越点选取：取离车最近那一段，用其两端点求中点与法向，在墙前留安全余量作为起飞/跨越点。
- `split_threshold_m` 调小可贴合更多拐点；`merge_collinear_deg` 控制碎段合并力度。

## 构建说明

按双设备工作流，本地不执行 `colcon build`；用 syncpi 传到开发板后在板上：

```bash
colcon build --packages-select obstacle_detector_pkg
```
