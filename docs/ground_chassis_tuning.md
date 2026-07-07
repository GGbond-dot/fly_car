# 飞车地面底盘 控制调参 & 调试记录

> 状态:根因已坐实、固件+ROS 两侧修法已写入代码(git,未编译),**待上板烧录/编译实测调参**(2026-07-07)。
> 本文是"跑动不稳定"问题的诊断 + 修法 + 调参手册,与[地面底盘与飞越](ground_chassis_and_flyover.md)配套(那篇讲地空互斥,本篇讲跑得稳)。

---

## 一、现象与根因(2026-07-07 用日志坐实)

**现象**:走方形时第一条边稳,换向后每条边**画龙 / 超调严重**,车"一顿一冲"。

**诊断手段**:`diff_drive_controller` 内建 CSV 日志(见 §五),跑方形回放。

**决定性证据**:命令角速度 `w ≤ 0.7 rad/s`,但**实际 yaw 角速度暴到 2~3.5 rad/s**,而且**反向打满 `w=−0.7` 车还在继续正转**,单个直角**超调 ~70°**。上层控制器该做的都做了(提前收油、提前反打),是**底盘不听命令**。

**根因链**(固件低速执行力问题,不是上层控制律问题):
1. 低速命令(原地转/近点微调,单轮才 ~5cm/s)**顶不动静摩擦**,轮子憋住不转;
2. 憋住期间,固件**增量式速度环** `ki*error` 逐拍累加,把 output 一路爬到 **~70% PWM**(正好是这套底盘突破静摩擦所需,满量程 10000);
3. 静摩擦一旦突破,轮子带着攒满的 PWM **猛冲**,转速远超小目标 → 暴甩、冲过头;
4. 冲过头后 `|e_h|>align_gate` → `v=0` 原地 spin → 再次憋死 → 再暴冲……
   **= stick-slip + 积分饱和释放的极限环,幅度巨大。**

> 圆角航点(`test_ground_square_round`)也没救:航点排太密(弧上每点 ~13cm、`pos_tol=5cm`),指向最近近点的方位角对几厘米位置误差**超敏**,仍逼出低速微调 → 同一个憋死暴冲。所以换汤不换药。

---

## 二、两层控制结构(改哪层、要不要烧录)

| 层 | 在哪 | 内容 | 改"值" | 改"逻辑" |
| --- | --- | --- | --- | --- |
| **A 固件内环** | SR5E1E3 板 `ZFEVB_SR5E1E3_Fly_Car/` | 轮速 PID + 前馈 + 抗饱和 + 加速度限幅;几何/编码器 | `$SET,…`+`$SAVE` 串口在线调,**不烧** | 必须**重新烧录** |
| **B ROS 外环** | 香橙派 `ground_chassis_pkg` | `diff_drive_controller`(carrot-chasing + 航向 PI)、align_gate、控制点偏移 | launch 参数 | 香橙派 `colcon build`,**不涉及烧录** |

**核心结论**:烧一次固件后,所有"数值"都从香橙派串口 `$SET`+`$SAVE` 调,**不用再烧**;只有改固件算法结构才需第二次烧。ROS 侧一律 `colcon build`,与烧录无关。

---

## 三、已实施的改动(git,未编译)

### 3.1 固件(`ZFEVB_SR5E1E3_Fly_Car/.../user_code/modules/`,一次烧录含全部)

| # | 文件 | 改动 |
| --- | --- | --- |
| 1 | `motor_control.c` | **静摩擦+黏性前馈**:目标 rpm 非零时叠加 `sign(rpm)*ff_static + ff_slope*rpm`,**左右分设**(补"左轮比右轮快"的不对称)。让低速命令立刻能动,积分不必爬到 70% 才突破 → 从源头消暴冲。 |
| 2 | `pid.c` | **条件积分抗饱和**:输出饱和时冻结积分项(防御性第二道;暴冲主要发生在 70%<满量程,主力是前馈)。航向环 ki=0 不受影响。 |
| 3 | `chassis.c` | **加速度斜率限幅**:接上原本定义了却没用的 `max_accel_v/w`,每拍朝目标 v/w 爬 `max_accel*0.01`,平滑 spin→直行突变。`$STOP`/断流仍立即停(clear 时清零斜率状态)。 |
| 4 | `param.c/.h` | 加 `ff_left/right_static/slope`、复用 `max_accel_v/w`;`param_set_ff/param_set_accel`;**`CAR_PARAM_VERSION` 2→3**。 |
| 5 | `protocol.c` | 新命令 `$SET,FF,L/R,static,slope`、`$SET,ACCEL,v,w`。 |

> ⚠️ **版本号 2→3**:烧后首启会发现 flash 里旧参数版本不符 → **自动恢复默认值**。你之前用 `$SET,PID` 存进 flash 的速度环增益会被清成默认(kp=14/ki=1.1),**烧后要重设一遍并 `$SAVE`**。

### 3.2 ROS(`ground_chassis_pkg`)

| 文件 | 改动 |
| --- | --- |
| `src/diff_drive_controller.cpp` | 航向环 **P→PI**:`w = kp_w*e_h + ki_w*∫e_h`,消左右轮恒定/缓变速度差导致的直行跑偏;带积分限幅 + 多路清零(大误差原地转/到点拧yaw/失能/超时/丢定位)防 windup。**内建 CSV 调参日志**(见 §五)。 |
| `launch/ground_chassis.launch.py` | `v_max_mps=0.25`(硬限速 25cm/s)、`kp_w=1.0`、`w_max_rps=0.7`、`align_gate_deg=60`、`ki_w=0.3`、`iw_limit_rps=0.3`、`ctrl_offset_x_cm=10.5`、`log_csv_path`(采集时填目录)。 |

---

## 四、烧一次 + 全串口在线调参(操作流程)

烧完这一次固件后,以下全部从香橙派串口(如 `scripts/vw_probe.py` 或任意串口工具)在线调:

```
$MODE,CONFIG                 # 进配置模式
$SET,PID,L,<kp>,<ki>,<kd>    # 轮速环 PID(烧后被恢复默认,先重设)
$SET,PID,R,<kp>,<ki>,<kd>
$SET,WHEEL,<直径m>,<轴距m>    # 几何标定(见 §六)
$SET,FF,L,<static>,<slope>   # 静摩擦/黏性前馈(见 §七)
$SET,FF,R,<static>,<slope>
$SET,ACCEL,<v加速度>,<w加速度> # 加速度限幅,0=不限(阶跃)
$SET,MAX_RPM/MAX_V/MAX_W,<值>
$SAVE                        # 持久化到 flash(不存则重启丢失)
$MODE,VW                     # 回 VW 模式让底盘桥驱动
```

---

## 五、诊断日志(CSV)与回传

- **打开**:`ground_chassis.launch.py` 里 `log_csv_path` 填目录(以 `/` 结尾),如 `/home/orangepi/kian_flycar/test_log/`。每次启动自动生成时间戳文件 `ddc_YYYYmmdd_HHMMSS.csv`(跑一次一个,不覆盖)。留空=关。
- **回传**:板上 `scripts/send_log.sh`(默认传最新一个;`-n N` 传最新 N 个;`--all` 全传)→ 回传到电脑 `kian@192.168.10.116:~/kian_flycar/test_log/`(目标可用 `DEST_USER/DEST_HOST/DEST_DIR` 环境变量覆盖;首次免密可 `ssh-copy-id`)。
- **列**:`t_s, phase(chase/align), d_cm, e_h_deg, integral, i_term_rps, v_mps, w_rps, yaw_deg, self_x, self_y, tgt_x, tgt_y`。
- **怎么看**:
  - `e_h_deg` 直行时是否收敛到 0(跑偏);
  - `integral`/`i_term_rps` 积分有没有把轮差补上、有没有撞 `iw_limit`;
  - **实际 yaw 角速度**(相邻 `yaw_deg` 差 / `dt`)是否 ≤ `w_max`——前馈生效后那个 2~3 倍暴甩应消失;
  - `v_mps` 验证限速 ≤0.25;`self_x/y`+`tgt_x/y` 可重画轨迹。

---

## 六、几何/编码器标定(换电机后必做,地基)

换了电机后 `wheel_diameter_m=0.072`、`wheel_base_m=0.136`、`encoder_ppr=255`、`max_rpm=300` 大概率没跟上。日志里直线实测偏慢(~0.17 vs 命令 0.25)就是信号。**尺子歪了,FF/PID 都白调。**

1. **轮径**:实心轮不压扁,卡尺量直径 → `$SET,WHEEL,<直径>,<轴距>`。
2. **核对 ppr(卡尺量不到)**:`$VW` 命令走已知距离(如 2m),卷尺量**实际走了多远**。实际≈命令 → ppr 没问题;差 >5% → 把残差**折进 `wheel_diameter`**(等比缩放,再 `$SET,WHEEL` 改一点,不用动 ppr、不用烧)。
3. **轴距**:量两轮接触中心距先填;再**原地转已知角度**核对实际转了多少度。

---

## 七、调前馈 `ff_static`(消暴冲的主力)

- 默认 `ff=0` = 旧行为,**必须调起来才有效**。
- 从小往大试(2000→4000→5000…)。这套底盘 ~70% PWM(≈7000)才突破静摩擦,`ff_static` 目标就在**略低于 7000**:设到"低速命令下轮子顺滑起步、不再憋一下再窜"为止,剩下让 PID 补。
- **左右分调**:你观察"左轮比右轮快",就把左轮 `ff_static` 调得比右轮略小;或跑纯直线命令看哪边先窜,微调到两轮同时起步、走直。
- `ff_slope`(黏性)先留 0;低速顺了、高速还偏软再加一点点。
- `target_rpm=0` 时前馈=0(不会自己爬行),安全。

---

## 八、待优化 TODO(重测后按需)

| 项 | 层 | 说明 |
| --- | --- | --- |
| `align_gate` 硬砍速 → 最小爬行 | ROS | 现 `|e_h|>60°` 就 `v=0` 原地 spin → 又进死区。改成保留最小爬行速度(~0.05m/s),边挪边转不停,治极限环。 |
| 上层改 **pure-pursuit 前视** | ROS | 现只盯**单个最近目标点**算方位角,近点超敏(圆角密航点震荡的根)。车端 `car/follower_pkg` 已有"沿轨迹前视一段弧长取 carrot",可搬过来。 |
| 航向环加 D / 曲率前馈 | ROS | FF 把底盘拉正后若还有余振:加 D 阻尼;或走弧时前馈名义 w,P/PI 只微调。 |
| `chassis_update_10ms` 左右轮独立钳位 | 固件 | 超 `max_rpm` 时左右各自钳位会扭曲 v/w 比例(转弯半径变形),应等比缩放。低速很少触发,小 bug。 |

---

## 九、相关文件速查

- 固件:`ZFEVB_SR5E1E3_Fly_Car/.../user_code/modules/{motor_control,pid,chassis,param,protocol}.c/.h`
- ROS:`src/ground_chassis_pkg/src/diff_drive_controller.cpp`、`launch/ground_chassis.launch.py`、`scripts/{chassis_bridge.py,send_log.sh}`
- 测试:`src/my_launch/launch/test_ground_square.launch.py`(直角方形,诊断跑偏用)、`test_ground_square_round.launch.py`(圆角)
- 诊断:`scripts/vw_probe.py`(不依赖 ROS 直接串口 $VW/$PWM/$GET,STATUS)
