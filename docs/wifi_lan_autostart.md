# 双机局域网 + Wi-Fi 自启动（飞车侧）

> **⚠ 方案变更（2026-06-14）：弃用「香橙派自建热点」，改用独立路由器组网。**
> 原因：orangepi 板载 wifi 当 AP 时**信号太弱，实测无法稳定使用**。
> 决定：用一台独立无线路由器建局域网，车与飞车都作为**普通客户端（STA）连同一路由器**。
> 影响：① 车端不再发 AP → `wifi_ap_manager`、AP→STA 驱动坑、标志位机制**全部不再需要**；
> ② 两块板都退化为"连路由器 wifi"，可直接靠 nmcli autoconnect 或一个极简连接脚本；
> ③ 信号/覆盖问题由路由器解决。
> 详见下方「七、路由器新方案（待落地）」。本文以下「一~六」是旧自建热点方案的记录，**飞车作为 STA 连接的那部分逻辑（静态 IP、UDP、域隔离）可平移到路由器方案**，AP 相关部分作废。

车与飞车通过同一局域网做跨机通信（飞车位姿 UDP → 车跟随）。车端对应文档见 `../../car/docs/wifi_lan_autostart.md`。

## 一、拓扑与地址

车板单网卡当 AP（基站/网关/DHCP），其余设备当客户端连入。

| 设备 | 角色 | IP | 由谁配 |
| --- | --- | --- | --- |
| 车板 | AP / 网关 / DHCP（`ipv4.method=shared` 自带 DHCP+NAT） | `192.168.50.1` | `car` 的 `wifi_ap_manager` |
| 飞车板 | STA（静态） | `192.168.50.2` | 本项目 `wifi_sta_manager` |
| 监控 PC | STA（DHCP 自动，`.10` 起） | `192.168.50.x` | 车 AP 的 DHCP |

- 热点：开放网络（无密码），SSID `OPi_ROS2_TEST`。车端建 AP 时主动移除 `802-11-wireless-security`。
- 飞车静态 `.2` 在 DHCP 池（`.10` 起）之外，不冲突。
- PC 用 NoMachine 连 `.1`/`.2` 远程观察两块板（NoMachine 是系统级软件，与 ROS 无关）。

## 二、关键认知

- **UDP 跨机传输与 `ROS_DOMAIN_ID` 无关**。`pose_sender` 走原生 socket（`192.168.50.1:8888`），不经 DDS。两块板的 `ROS_DOMAIN_ID` 仍必须错开（车=10，飞车取非 10），那是为了防各自的 `/scan`、`/tf`、`map`/`laser_link` 在同一 LAN 内被 DDS 多播发现而串台、污染 TF 树。组网后这条隔离比以前更要盯紧。
- **单网卡 AP/STA 互斥**：`wlan0` 同时只能发 AP 或连一个 wifi。车板开 AP 期间没有外网（正常，开发/比赛都不需要车板外网；PC 连热点走 `192.168.50.1` rsync 即可）。
- **车板 AP→STA 驱动切不干净**（orangepi5max 板载 wifi）：运行中从 AP 模式切回连上网 wifi 会出 `Secrets were required` / `network could not be found` 等自相矛盾报错。故**车端不做运行时切换**，靠标志位 + 重启（见车端文档）。**飞车是纯 STA，STA↔STA 切换无此坑，可实时自动择网**。

## 三、`wifi_sta_manager` 功能包

连入车端开放热点的客户端，与车端 `wifi_ap_manager` 对称。

- 节点 `sta_manager`，action `connect` / `disconnect` / `status`。
- 参数：`ssid`(默认 `OPi_ROS2_TEST`)、`interface`(`wlan0`)、`connection_name`(`OPi_ROS2_JOIN_AP`)、`ipv4_method`(`manual`/`auto`)、`ip_cidr`(`192.168.50.2/24`)、`gateway`(`192.168.50.1`)、`rescan`。
- 用法：`ros2 launch wifi_sta_manager connect_ap.launch.py`（DHCP 模式加 `ipv4_method:=auto`）。connect 是一次性动作，节点退出后 nmcli 连接保持。
- `pose_sender` 的 `target_ip` 默认已改 `192.168.50.1`＝车 AP 固定地址。

## 四、自启动脚本（`fly_car/scripts/`）

仿 `kian_26fly/scripts/autostart_fly.sh` 套路的自包含单脚本：`SCRIPT_DIR/..` 自动解析 `WS_ROOT`、显式 `source` ROS+install、`set -o pipefail` 但不开 `set -u`、日志 `~/wifi_logs/wifi_sta.log`。**加到桌面「会话与启动」时命令直接填脚本绝对路径**（无需 .desktop/.service，登录用户身份、免 sudo）。

### `autostart_wifi.sh` — 自动择网（飞车专属逻辑）

每 `INTERVAL`（默认 5s）扫描一次，据车热点是否可见自动二选一：

- 扫到 `OPi_ROS2_TEST` → 连/保持**车 AP**（`OPi_ROS2_JOIN_AP`，静态 `.2`），并 ping `192.168.50.1` 自检，不通则重连。
- 扫不到 → 连/保持**默认上网 Wi-Fi** `UPLINK_CONN`，保证能上网/开发。

即：车一开机飞车几秒内自动进局域网；车关了飞车自动回上网。状态变化才记日志，不刷屏。

- **`UPLINK_CONN` 默认 `HUAWEI-GR18QG`**（飞车板的上网 wifi 名）；可用环境变量 `FLYCAR_UPLINK_WIFI` 覆盖，或直接改脚本。换板/换网时记得改。
- 其它可调环境变量：`WIFI_HEARTBEAT_SEC`（心跳周期）、`WIFI_LOG_DIR`（日志目录）。

### `stop_wifi.sh`

`pkill` 掉 `autostart_wifi.sh` 心跳，停止后保持当前所连、不再自动切换。恢复自动择网：重跑 `./autostart_wifi.sh` 或重启。

## 五、上板验证步骤

1. 板上 `colcon build --packages-select wifi_sta_manager && source install/setup.bash`（本地不编译）。
2. 单独验 launch：车 AP 开着时 `ros2 launch wifi_sta_manager connect_ap.launch.py` → `ping 192.168.50.1` 通。
3. 前台跑 `./autostart_wifi.sh` 看日志：车开→"连局域网/局域网正常"，车关→"连默认上网 Wi-Fi"。
4. 通过后把脚本绝对路径加进「会话与启动」，重启验证。
5. 排查看 `~/wifi_logs/wifi_sta.log`。

## 六、风险 / 观察项

- 同一驱动两个 STA 间频繁切换偶尔可能抽风；若发现切换后没拿到 IP，再加"切后验证 IP，没有则 `nmcli radio wifi off/on` 复位"的兜底。
- `UPLINK_CONN` 名字必须与飞车板 `nmcli connection show` 里的连接名**完全一致**（含大小写/连字符），否则回连上网失败。

---

## 七、路由器新方案（已落地，2026-06-15 更新）

用一台独立无线路由器建局域网，车与飞车都作为 STA 连它。比自建热点简单得多。

**实际拓扑 / 地址（路由器网段 `192.168.10.x`）**

| 设备 | 角色 | IP |
| --- | --- | --- |
| 路由器 | AP / DHCP / 网关 | 路由器自身（按路由器后台） |
| 车板 | STA | `192.168.10.161` |
| 飞车板 | STA | `192.168.10.171` |
| 监控 PC | STA | DHCP |

**已落地的做法**
1. **两块板开机自动连路由器 wifi**（NetworkManager `connection.autoconnect yes`）。**直接绕过 `wifi_sta_manager` / `autostart_wifi.sh` / 标志位 / 心跳那一套**——这些是旧自建热点方案的残留，路由器自动连已替代其全部职责。相关包与脚本暂保留但**不再启动、不加入桌面自启动**，标记作废。
2. `pose_sender` 的 `target_ip` **已在代码里改成 `192.168.10.161`**（车板在路由器网的地址；飞车自身 `192.168.10.171`）。端口 `8888`，与车端 `leader_pose_receiver` 的 `udp_port` 一致。若 DHCP 导致车 IP 变动，联调时用 launch 参数覆盖：`ros2 launch pose_sender_pkg pose_sender.launch.py target_ip:=<车实际IP>`；或在路由器后台按 MAC 绑静态租约固定为 `.161`。
3. **`ROS_DOMAIN_ID` 隔离仍然必须**（车=10、飞车非 10，如 42）——同一局域网内 DDS 多播照样会互相发现。**作准方式：在各板 `~/.bashrc` 末尾 `export ROS_DOMAIN_ID=<值>`**（覆盖节点 + 所有 ros2 CLI 调试终端，单一真相源）。注意 UDP 位姿桥走原生 socket，不受域影响，照常通。

**保留可平移的部分**：UDP 位姿桥协议、域隔离、NoMachine 远程观察，都原样可用，只是"连谁"从车 AP 变成路由器。
