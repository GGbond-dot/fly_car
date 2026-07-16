# 飞车固定 L 地面轨迹调参采集脚本设计

## 目标

为飞车唯一的地面轨迹提供可重复执行的数据采集工具。每次测试只跑一遍固定 L，终点后停止；操作员把飞车搬回起点，再启动下一轮并更换参数。脚本不负责返程、起飞、YOLO、投放或跨机通信。

固定航点使用飞车 map 系、cm/deg：

```text
#1 (  0,   0, 0) yaw= 0.0
#2 ( 50,   0, 0) yaw=74.1
#3 (150, 350, 0) yaw=74.1
```

航点语义保持正式任务约定：#1→#2 固定 0° 直走；到 #2 停车并原地转到 74.1°；#2→#3 固定 74.1° 直走。移动与任务级 yaw 切换不得同时发生。

## 接入方式

新增 `fly_car/scripts/l_path_tuning.py`，在飞车板、`ROS_DOMAIN_ID=1` 下运行。脚本发布整条路线到 `/wildlife/waypoints`，复用正式链路：

```text
l_path_tuning.py
  → /wildlife/waypoints
  → route_target_publisher
  → /target_position
  → chassis_mux
  → diff_drive_controller
  → /cmd_vel
  → chassis_bridge
```

脚本不直接发布 `/target_position`，避免绕过航点推进、到点判定和地空仲裁。运行前要求现有 `patrol_ground.launch.py` 已启动；若缺少 `route_test_node`、`diff_drive_controller` 或 `chassis_mux`，脚本拒绝发车并给出缺失节点。

## 命令行

基础运行：

```bash
export ROS_DOMAIN_ID=1
python3 ~/kian_flycar/scripts/l_path_tuning.py
```

单轮覆盖热调参数：

```bash
python3 ~/kian_flycar/scripts/l_path_tuning.py \
  --set straight_kp_w=0.22 \
  --set straight_w_max_rps=0.16 \
  --set yaw_lpf_alpha=0.25
```

参数覆盖通过 `/diff_drive_controller` 参数服务完成。任一参数不存在、类型不合法或设置失败时，不发布路线。其它选项：

- `--output-dir`：输出根目录，默认 `~/kian_flycar/test_log/l_path_runs/`。
- `--timeout-s`：单轮超时，默认 120 秒。
- `--endpoint-stable-s`：终点稳定时间，默认 0.5 秒。
- `--dry-run`：只检查节点、TF 和参数，不发车。

## 采集数据

每轮创建 `YYYYMMDD_HHMMSS/`，包含：

- `samples.csv`：20 Hz 采样。
- `params.json`：发车前 `/diff_drive_controller` 参数快照与本轮 `--set` 覆盖。
- `summary.json`：机器可读统计。
- `summary.txt`：现场可直接查看的结论。

`samples.csv` 至少记录：

```text
t_s,segment,target_x_cm,target_y_cm,target_yaw_deg,
pose_x_cm,pose_y_cm,pose_yaw_deg,v_cmd_mps,w_cmd_rps,
heading_error_deg,cross_track_cm,ground_enable,flight_enable
```

`segment` 分为 `WAIT_START`、`LEG_1`、`TURN`、`LEG_2`、`DONE`。分段以 `/target_position` 当前目标判断，不按固定时间猜测。

每条直线分别统计：

- 持续时间和行驶距离。
- 平均/最大速度。
- 航向误差 RMS、最大绝对值、峰峰值。
- 横向误差 RMS、最大绝对值。
- 有效 `w` 反号次数：忽略 `|w| < 0.02 rad/s` 的噪声区。
- `w` 最大绝对值。

原地转统计转向时间、最大 yaw 过冲和最终 yaw 误差。

## 完成与安全

脚本使用 `map→laser_link` TF 判断终点：距 #3 不超过 12 cm、yaw 与 74.1° 相差不超过 5°，连续稳定 0.5 秒后完成并退出。正常完成后不发布返程或起飞命令。

以下情况立即中止：

- TF 超过 0.5 秒未更新或连续查询失败。
- `/flight_enable=true`。
- 单轮超时。
- 操作员 Ctrl-C。

中止时脚本把当前 TF 位姿作为单点地面保持航点发布一次，使正式执行器停止平移；随后由现有控制器目标超时、底盘桥看门狗和固件超时继续兜底。测试不得安装桨。

## 首轮基准参数

首轮以压制摆头为优先，速度保守：

```text
kp_v                       1.0
v_max_mps                  0.18
v_min_mps                  0.14

straight_kp_w              0.22
straight_w_max_rps         0.16
straight_yaw_deadband_deg  1.0
yaw_lpf_alpha              0.25

ki_w                       0.0
kd_w                       0.02
yaw_rate_lpf_alpha         0.4
w_slew_rps2                1.2
w_bias_rps                 0.0

kp_w                       0.60
w_max_rps                  0.70
w_min_rps                  0.40
align_gate_deg             30.0
pos_tol_cm                 5.0
yaw_tol_deg                3.0
```

其中 `straight_*` 只控制直线微调；`kp_w/w_max_rps/w_min_rps` 主要控制 #2 原地转向。`lookahead_dist_cm` 对本任务无效，因为正式巡逻 launch 固定 `lookahead_count=0`。

## 首轮后的调参顺序

一次只改一类参数：

1. 直线仍周期摆动：`straight_kp_w` 依次降为 0.18、0.14；必要时 `straight_w_max_rps` 降为 0.12。
2. `w` 反号很密但车头实际没明显转：死区增至 1.5°，或 `yaw_lpf_alpha` 降至 0.18。
3. 摆动变慢且幅度变大：说明滤波/斜率限制带来过多延迟；`yaw_lpf_alpha` 增至 0.35，或 `w_slew_rps2` 增至 1.8。
4. 只向一侧缓慢漂：先把死区降至 0.7°；仍漂再按 0.005 rad/s 步长调 `w_bias_rps`，不启用积分。
5. #2 转不到位：只提高 `w_min_rps` 或 `w_max_rps`；原地转过冲则反向调整，必要时把 `kd_w` 增至 0.03–0.05。

每次修改后重新跑完整 L，不在同一轮中途改参数，确保参数快照与数据一一对应。

## 验证

开发机只做 Python 语法和无 ROS 的统计单元测试，不运行 `colcon build`。板上验证顺序：

1. 不装桨。
2. 编译现有 ROS 包并启动 `patrol_ground.launch.py`。
3. 先执行 `--dry-run`。
4. 执行首轮基准参数。
5. 将完整轮次目录传回开发机分析。

