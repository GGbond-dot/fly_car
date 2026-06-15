# 飞车地面底盘 + 起飞/飞越/落地 设计

> 状态:代码完成(2026-06-14),待上板 `colcon build` + 实测调参。
> 本文是飞车"陆地差速行驶 ↔ 空中飞越"两套控制链如何**互斥切换**的设计,与[任务架构](coverage_flyover_mission_design.md)配套。

---

## 一、为什么需要这套

飞车是陆空两用:**地面行驶有独立的轮式底盘**(和地面车 `car` 同款 **SR5E1E3 差速板**,`/dev/ttyS6` @115200,`$VW` 文本帧),空中才用原有 STM32 全向飞控(`pid_control_pkg → uart_to_stm32`,`/dev/ttyUSB0` 二进制帧)。地面不是靠飞控贴地飞。

**硬要求:两套控制链严格互斥** —— 车跑时飞控 PID 绝不能运行,飞控起飞时轮子绝不能转。否则会出现"飞控在地面乱发速度""轮子在空中空转"等危险情况。

`pid_control_pkg` 是飞行功能,**只允许加一个使能标志位,不改其控制算法**;`uart_to_stm32` 零改动。

---

## 二、新增件(包 `fly_car/src/ground_chassis_pkg`)

| 节点 | 职责 |
| --- | --- |
| `diff_drive_controller` | 从 `car/follower_pkg` 搬来的差速跟踪控制器(carrot-chasing,不能横移)。输入 `/target_position` + `/ground_enable`,输出 `/cmd_vel`。`/ground_enable=false` 时持续发零速(轮子停)。 |
| `chassis_bridge.py` | 仿 car bridge 的 `$VW` 流式通道 + 看门狗,去掉舵机/离散命令。`/cmd_vel` → `$VW,v,w`,断流 0.5s 发 `$STOP`。 |
| `chassis_mux` | **地空互斥的唯一裁判**。按目标 z 与实测 `/height` 仲裁,latched 发布互斥的 `/ground_enable` 与 `/flight_enable`。 |

对 `pid_control_pkg` 的唯一改动:新增订阅 `/flight_enable`(标志位),`false` 时 `controlTimerCallback` 直接 return 且 reset 积分;默认 `true` 兼容无 mux 的纯飞行调试。

```
                              ┌─ /ground_enable → diff_drive_controller → /cmd_vel → chassis_bridge($VW) → SR5E1E3 地面板
/target_position ─► chassis_mux┤
   + /height ───────►          └─ /flight_enable → pid_control_pkg(标志位)→ /target_velocity → uart_to_stm32 → STM32 飞控
```

---

## 三、模式判据(chassis_mux)

两个 enable 由同一个 `Mode` 派生,**绝不会同时为真**。判据**非对称**(起飞快、降落稳):

| 转换 | 判据 |
| --- | --- |
| 起飞 GROUND→AIR | 目标航点 `z > z_threshold_cm`(默认 20)→ 立即切空中,让飞控马上接管拔高 |
| 降落 AIR→GROUND | 目标 z 低 **且** 实测 `/height ≤ land_height_cm`(默认 15cm)才切地面 |
| 启动 SAFE→x | 收到首个目标前两者皆 false;首次激活直接生效(无缓冲) |

统一规则:`AIR 当 (目标z>阈值) 或 (实测高度>land_height_cm);否则 GROUND`。

> **为什么降落要看实测高度**:落点航点目标 z=4cm,但飞机此刻还在 100cm。若只按目标 z 切地面,会在半空中关掉飞控、轮子空转 → 飞机失控掉落。所以降落全程由飞控控制下降,**真正贴地(/height≤15)才交给轮子**。`/height` 由 `uart_to_stm32` 回传飞控实测高度。

### 切换缓冲(稳定停顿)

任何起飞/降落切换前,先进**缓冲态**:两套都停(`ground_enable=false` + `flight_enable=false`),停 `settle_s`(默认 2s)再使能目标模式。

```
起飞:地面跑 → 目标z>20 → 【停2s 轮停、飞控不发】→ 飞控使能 → 起飞
降落:飞越中 → /height≤15 → 【停2s 都停】→ 地面使能 → 跑
```

缓冲期内判据若抖动翻回,自动改向新目标并重新计时,不卡中途。

---

## 四、起飞/飞越/落地的航点机制(activity_control_pkg)

核心思路(用户定):**xy 航点开跑前一次性全发布,z 只是模式指示**。覆盖航点 z=巡航低值(地面),"飞不飞"由障碍检测决定。

### 4.1 起飞 —— 原地垂直 + 全局 z 覆盖

`obstacle_decision` 订阅 `/detected_obstacle`,当墙逼近到**以车为心半径 `approach_threshold_m`(默认 0.6m)**内(`path_dist ≤ 0.6`):

1. `RouteTargetPublisher::insertNext([当前xy, z=flyover_z(100)])` —— 在当前航点前插一个**当前位置**的高 z 点 → **原地垂直起飞**(xy 不动,只升)。
2. `RouteTargetPublisher::setFlightMode(true, 100)` —— **全局 z 覆盖**:之后发布/到达判定都把 z 顶成 100,**xy/yaw 仍用原航点** → 沿原来的 xy 在空中飞越墙。

任务只有这一道墙,默认 `single_shot`:触发一次后停发 `/obstacle_detect_enable` 自锁。

### 4.2 落地 —— 航点标志位

`Target` 带 **`land_after` 标志位**。你在**落点航点**上打标记(`coverage_generator` 的 `land_after_indices` 参数指定下标;以后自写航点列表直接设 `true`)。

飞到带标志的航点后,`RouteTargetPublisher`:
1. `setFlightMode(false)` —— 退出 z 覆盖(落点用真实低 z)。
2. 插一个**同 xy、z=`land_z_cm`(默认 4)**的下降点 → 原地垂直下降。

下降全程仍由飞控控制(mux 看实测高度未落地不切),`/height≤15` 才切回轮子。

### 4.3 isReached 容忍度方向(重要)

- **地面航点 z 容忍松**(`ground_z_tol_cm`,默认 30):忽略地面高度噪声,走到 xy(+yaw)就算到。
- **空中航点 z 容忍紧**(`air_z_tol_cm`,默认 8):高度必须到位。
- 地面那个"松但有限"的好处:从 100cm 降回时 z 误差很大,不会误判到达,得真降到地面附近才推进。

---

## 五、完整时序

```
地面跑(z=4,松容忍,差速底盘)
  │ 墙<0.6m:insertNext(当前xy,z=100) + setFlightMode(true)
  ▼
mux:目标z>20 →【停 settle 秒】→ flight_enable=true → 原地垂直起飞
  ▼
沿原 xy 空中飞越(z=100 覆盖,紧容忍)
  │ 到带 land_after 的落点:setFlightMode(false) + 插同 xy 下降点(z=4)
  ▼
飞控控制下降 → 实测 /height≤15 →【停 settle 秒】→ ground_enable=true → 落地继续跑
```

---

## 六、关键参数

| 参数 | 默认 | 节点 | 说明 |
| --- | --- | --- | --- |
| `approach_threshold_m` | 0.6 | obstacle_decision | 墙逼近触发起飞(以车为心半径 m) |
| `flyover_z_cm` | 100 | obstacle_decision | 越障飞行高度 |
| `z_threshold_cm` | 20 | chassis_mux | 目标 z 高于此算空中 |
| `land_height_cm` | 15 | chassis_mux | 实测高度低于此才切回地面 |
| `settle_s` | 2.0 | chassis_mux | 起飞/降落切换前稳定停顿 |
| `ground_z_tol_cm` / `air_z_tol_cm` | 30 / 8 | RouteTargetPublisher | 地面松 / 空中紧 z 容忍 |
| `land_z_cm` | 4 | RouteTargetPublisher | land_after 落点高度 |
| `land_after_indices` | [] | coverage_generator | 哪些航点打落地标志 |
| 底盘串口 | `/dev/ttyS6` @115200 | chassis_bridge | 与 car 同款 SR5E1E3 |

> 场地边界/行距、PID 增益、上述高度阈值均为占位/待实测标定。按双设备工作流,本地不编译,上板验证。

---

## 七、待办 / 风险

- **落地判据依赖 `/height`**:若飞控不发 `/height` 或想用别的落地判据,改 `chassis_mux` 的 `physically_air` 逻辑。
- **切换瞬态**:目标与 enable 走不同话题,切换那一拍 diff 控制器可能短暂收到高 z 目标(它只用 xy);缓冲态已先把两者停掉,基本消除。
- **多道墙**:当前 `single_shot` 只处理一道墙(任务设定如此);若要多道,落地后需重新开 `/obstacle_detect_enable` 并解锁。
- 占位参数全部待上板实测。
