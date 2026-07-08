# 飞车 灾区两次投放 任务编排 设计 / 实现

> 状态:**地面段实现完成、待上板联测**(2026-07-08)。飞行段状态机已写但当前 `ground_only=true` 暂停(到起飞点即停,不起飞)。
> 坐标为现场标定值;跨机改原生 UDP(弃用 domain_bridge);地面段加了 YOLO 视觉人机确认握手。

底层执行链(航点执行、地空互斥、起降)沿用 RouteTargetPublisher / chassis_mux / pid_control_pkg,本文只讲**任务层 + 跨机 + 视觉握手**。

---

## 一、总体流程(当前:地面段,飞车不飞)

坐标系:Cartographer `map`,REP-103(+x 前 / +y 左 / yaw 逆时针为正),飞车起点 (0,0) 朝 +x。**右转 90° = yaw −90**(−y 方向)。单位 cm。

```
飞车直行到检测点 (245,0) 停下           [GROUND_TO_DETECT]
  └ YOLO 看到 rescuee → 飞车桥发标志(带类别)给车
  └ terminal 语音播报"识别到难民,是否投送补给?" + 平板弹确认
  └ 用户点"投送" → terminal 发确认 → 飞车收到          [WAIT_CONFIRM]
右转 −90 弧转到 (265,-30) 投货1(舵机1)  [GROUND_TO_DROP1 → DROP_1]
继续直行(−y)到起飞点 (265,-97)          [GROUND_TO_TAKEOFF]
发 /resupply_request 叫车,停等          [WAIT_RESUPPLY]
  └ 车开到对接位 (306,-64) 推货 → 退开20cm → 发 /resupply_done → 开回起点
飞车收到 done → 停(ground_only,不起飞)  [DONE]
```

**飞行段(ground_only=false 时启用,当前暂停)**:原地起飞到 z=100 → 保持 yaw −90 平飞到难民2 上方 **(-8,-270)** → (第二轮 terminal 询问确认)→ 降到 z=50 投货2 → 沿机头前进 100cm → 原地垂直降落。用 `land_after` 实现"先平飞后垂直落,不斜切"。

---

## 二、架构:同进程组合

`relief_drop_mission_main.cpp`:`RouteTargetPublisherNode`(航点队列 + 到达推进 + z 编码起降)+ `MissionSequencerNode`(状态机,持 route 指针调 addTarget/setFlightMode),单线程 executor,route/mux/pid **零改动**复用。

### 状态机

| # | 状态 | 出口门 | 进入动作 |
|---|---|---|---|
| 0 | `INIT` | 启动延时 | — |
| 1 | `GROUND_TO_DETECT` | 队列排空(到 (245,0)) | 摄像头舵机→120;倒货舵机→复位90;addTarget(检测点)停下 |
| 2 | `WAIT_CONFIRM` | 收到 `/terminal_confirm` | (等 terminal 人机确认;`wait_terminal_confirm=false` 可跳过) |
| 3 | `GROUND_TO_DROP1` | 队列排空(到 (265,-30)) | addTarget(投货1落点),右转弧转 |
| 4 | `DROP_1` | 投放延时到 | 舵机1 open(180)→停→close(90) |
| 5 | `GROUND_TO_TAKEOFF` | 队列排空(到 (265,-97)) | addTarget(起飞点) |
| 6 | `WAIT_RESUPPLY` | 收到 `/resupply_done` | 发 `/resupply_request` 叫车;停等 |
| — | **ground_only=true** | — | → 直接 DONE(不起飞) |
| 7 | `TAKEOFF` | 队列排空(拔高到 z=100) | 摄像头→180;setFlightMode(true,100)+原地垂直拔高 |
| 8 | `FLY_TO_WP2` | 队列排空(到难民2上方) | 逐个 addTarget(飞行航点,yaw 全程=起飞 yaw −90) |
| 9 | `DESCEND_2` | 队列排空(降到 z=50) | setFlightMode(true,50) 原 xy 悬停下降(不落地) |
| 10 | `DROP_2` | 投放延时到 | 舵机1 open→close |
| 11 | `FORWARD_LAND` | 队列排空(前进后垂直落地) | setFlightMode(true,z_forward)+addTarget(沿机头前进点,**land_after=true**) |
| 12 | `DONE` | — | 结束 |

> **视觉握手(WAIT_CONFIRM)**:检测点在难民正前方,飞车走到即停;YOLO 看到 rescuee 触发 terminal 询问,用户确认才右转投货。**空中难民(FLY_TO_WP2→DESCEND_2 之间)的第二轮询问待开飞行段时补第二道确认门**,接口与地面同。
> **FORWARD_LAND 用 `land_after`**:飞行模式到达前进点后 route 自动退飞行模式、原地垂直下降到 `land_z`,天然"先平飞后落"。

---

## 三、跨机通信:原生 UDP(弃用 domain_bridge)

**两板都不设 `ROS_DOMAIN_ID`(都在默认域 0)**,且路由器基本挡 DDS 多播(位姿桥 pose_sender 走原生 UDP 正是此因)。若靠 DDS 单播 peer 跨机会把 `/scan`、`/tf`、`map` 一起暴露 → 两边 Cartographer 同时跑必串台污染 TF。所以**跨机一律走原生 UDP,完全不碰 DDS 发现,scan/tf/map 天然隔离**。

两侧各一个 `xmachine_bridge`(飞车 `activity_control_pkg` / 车 `follower_pkg`),把本地 ROS 话题 ↔ UDP 翻译,mission_sequencer / obstacle_detector / yolo / terminal **零改动**。

| magic | 方向 | 载荷 | 本地话题(收端发布/发端订阅) |
|---|---|---|---|
| `FC02` | 飞车→车 | bool | `/resupply_request` 到起飞点叫车 |
| `FC03` | 飞车→车 | 变长 float | `/detected_obstacle` 折线障碍(发一次,burst) |
| `FC04` | 车→飞车 | bool | `/resupply_done` 补给完成、已退开 |
| `FC05` | 飞车→车 | uint8=类别 | `/rescuee_detected`(Int8:0无/1/2)YOLO 识别标志 |
| `FC06` | 车→飞车 | bool | `/terminal_confirm` terminal 人机确认 |

- 包:`BoolPacket{magic u16, id u16, value u8}`(5B,REQ/DONE/RESCUEE/CONFIRM);`ObstHeader{magic,id,count u16}`+count×f32(OBST)。**改一处两侧同步改。**
- 端口:车 bind `8890` 收飞车 req/obst/rescuee;飞车 bind `8891` 收车 done/confirm。pose 桥的 `8888` 独立勿混。IP:飞车 `.171` / 车 `.161`。
- 一次性事件 burst 重发(`resend_count=30 @10Hz≈3s`)扛丢包,收端按 id/内容幂等去重。
- YOLO 门:飞车桥订 `/yolo_detector/detections`,conf≥`yolo_conf_thresh`(0.25)+连续 `yolo_debounce_frames`(2)帧命中判"识别到",持续重发 FC05(value=类别)。

---

## 四、补给握手(车侧)

飞车到起飞点发 `/resupply_request`。车侧 `resupply_node` 状态机:
`IDLE→PREEMPT(/follow_enable=false 抢占)→GO_DOCK(开到对接位 306,-64,-90)→PUSH_OUT/PUSH_RETRACT(推货舵机)→RETREAT(退开20cm 306,-44)→RETURN_HOME(发 /resupply_done + 开回起点 0,0)→DONE`。

- 推货用车上**推货舵机(1号)**:发 `car_movement`(String)给桥 `orangepi_to_carv2`,`"5"`=推出送货(**70°**)、`"6"`=收回(初始伸直 **180°**),角度桥侧 `--servo-extend/retract-deg` 标定。**不是车往前顶货。**
- 退开只走前进/小后退(carrot-pursuit 不擅长倒车),`RETURN_HOME` 是前进开回起点。

---

## 五、视觉 + 视频(terminal)

- **投放舵机(飞车,1号,ttyS3)**:倒货 open=**180**、复位 close=**90**(实测);任务起步自动复位到 90 关箱。测试脚本 `fly_car/scripts/servo_test.py`。
- **摄像头舵机(飞车,2号)**:地面 120 / 飞行 180,由 sequencer 发 `/servo_cmd` → chassis_bridge 转 `$SERVO`。
- **YOLO**:`yolo_detector_pkg`(YOLOv5s + RK3588 NPU/RKNN),读 /dev/video0,发 `/yolo_detector/detections`(每目标 6 float:cls,conf,x1,y1,x2,y2;cls 0=rescuee1/1=rescuee2)+ MJPEG 推流 `:8080`。
- **视频给平板**:飞车 MJPEG `http://192.168.10.171:8080/stream.mjpg`(帧带识别框)。car 板 terminal(`kian_ai_0001`)网页里 `<img src=...>` 直接嵌(三者同局域网,平板浏览器直连飞车拉流)。**不走 ROS/DDS。**
- **人机确认**:terminal 订 `/rescuee_detected`(Int8)→ 语音播报 + 平板弹"投送/取消" → 用户点投送 → 发 `/terminal_confirm`(Bool)→ 飞车右转投货。terminal 与车节点同在默认域 0。

---

## 六、Launch / 测试

| launch | 板 | 内容 |
|---|---|---|
| `my_launch relief_drop_ground.launch.py` | 飞车 | 地面段全套:fly_carto + ground_chassis + relief_drop_mission + xmachine_bridge(不含飞控) |
| `car_launch resupply_ground.launch.py` | 车 | car_carto + diff_drive + orangepi_to_carv2 + resupply + xmachine_bridge(不含 follower) |
| `activity_control_pkg comm_test.launch.py` | 飞车 | 只起 xmachine_bridge,手动 pub/echo 验 5 条跨机信号 |
| `follower_pkg comm_test.launch.py` | 车 | 同上(对端) |
| `my_launch yolo_comm_test.launch.py` | 飞车 | yolo_detector + xmachine_bridge,验真实识别→标志→车 + 视频流 |

各 comm_test / yolo_comm_test 的文件头注释里有逐条测试命令。

---

## 七、关键参数(现场标定值,在 launch 里改)

| 参数 | 值 | 含义 |
|---|---|---|
| fwd (x,y,yaw) | 245,0,0 | 检测点(YOLO 看货处),停下等确认 |
| drop1 (x,y,yaw) | 265,-30,-90 | 右转弧转投货1 落点 |
| takeoff (x,y,yaw) | 265,-97,-90 | 固定起飞点;飞行段全程保持 yaw −90 |
| fly_waypoints | -8,-270 | 难民2 上方(投货2 下降点) |
| flight_z / z_drop2 / z_forward | 100 / 50 / 50 | 巡航 / 悬停投放 / 前进平飞高度 |
| forward_after_drop2 | 100 | 投货2 后沿机头前进距离,到点垂直落 |
| servo_open / close | 180 / 90 | 倒货舵机1 投放 / 复位(实测) |
| camera 2 / ground / flight | 2 / 120 / 180 | 摄像头舵机及地面/飞行角 |
| ground_only | true | 当前只跑地面段,到起飞点即停 |
| wait_terminal_confirm | true | 到检测点等 terminal 确认才右转投货 |

---

## 八、待办 / 风险

- **地空互斥未合并测**:ground_chassis(含 mux)+ 飞控栈(uart/pid)从没在一条 launch 里跑过;开飞行段时 pid 要 `flight_enable_default=false` 让 mux 使能,arbitration 上板重点验。当前 ground_only 暂避开。
- **空中第二轮询问**:飞行段 FLY_TO_WP2→DESCEND_2 之间补第二道 WAIT_CONFIRM(接口同地面);同时确认门要改成每轮可复位(现地面单轮 set-once)。
- **域**:全默认 0。注意 `car/orangepi_to_car/launch/flycar_system.launch.py` 遗留硬设域 10,别让 terminal 走它而顶到域 10(与桥对不上);`echo $ROS_DOMAIN_ID` 两边都空即通。
- **弧形右转**靠追点成弧,弧半径由 drop1 相对位 + 控制器 lookahead/kp_w 决定,可能要配 lookahead_count 调顺。
- **退开/回起点**:carrot-pursuit 追后方点会先掉头;退距小,上板看现象,必要时改开环 `$VW` 负速直退。
- 投放停留 `t_drop_s`、YOLO 触发点等仍待现场微调。
