#!/usr/bin/env bash
# kian_flycar 飞车(STA)Wi-Fi 开机自启 + 自动择网 — 仿 kian_26fly/scripts/autostart_fly.sh 套路。
#
# 加到桌面"会话与启动":命令直接填本脚本绝对路径即可(不需要 .desktop/.service)。
# 逻辑(飞车是纯客户端,STA<->STA 切换无 AP 驱动坑,可实时自动判断):
#   每 INTERVAL 秒扫描一次:
#     - 扫到车热点(AP_SSID) -> 连/保持局域网(连车 AP,静态 192.168.50.2,供跟随)
#     - 扫不到          -> 连/保持默认上网 Wi-Fi(UPLINK_CONN),保证能上网/开发
# 停止: 同目录 stop_wifi.sh(停心跳)。日志: ~/wifi_logs/wifi_sta.log。
#
# 注意:不要开 set -u(nounset)。ROS 的 setup.bash 会引用未定义变量,开了会直接报错退出。
set -o pipefail

# ---- 路径解析:脚本在 <ws>/scripts/ 下,ws 根 = 上一级(不写死路径)----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LOG_DIR="${WIFI_LOG_DIR:-$HOME/wifi_logs}"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/wifi_sta.log"

IFACE="wlan0"
AP_SSID="OPi_ROS2_TEST"                       # 车端热点 SSID(open_ap.launch.py 默认)
AP_CONN="OPi_ROS2_JOIN_AP"                    # 连车 AP 用的连接名(connect_ap.launch.py 默认)
UPLINK_CONN="${FLYCAR_UPLINK_WIFI:-HUAWEI-GR18QG}"  # 飞车板"默认上网 Wi-Fi"连接名(可用 FLYCAR_UPLINK_WIFI 覆盖)
PEER_IP="192.168.50.1"                        # 车 AP 固定地址(局域网连通性自检)
INTERVAL="${WIFI_HEARTBEAT_SEC:-5}"           # 心跳周期(秒)

source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"

log() { echo "[wifi-sta $(date '+%F %T')] $*" | tee -a "$LOG"; }

is_active()    { nmcli -g NAME connection show --active 2>/dev/null | grep -qx "$1"; }
ssid_visible() { nmcli -t -f SSID device wifi list 2>/dev/null | grep -qx "$AP_SSID"; }
peer_ok()      { ping -c1 -W1 "$PEER_IP" >/dev/null 2>&1; }
join_ap()      { ros2 launch wifi_sta_manager connect_ap.launch.py action:=connect >>"$LOG" 2>&1; }
join_uplink()  { nmcli connection up "$UPLINK_CONN" >>"$LOG" 2>&1; }

trap 'log "autostart_wifi 退出"; exit 0' INT TERM

log "=== autostart_wifi 启动: ws=$WS_ROOT ap_ssid=$AP_SSID uplink='$UPLINK_CONN' interval=${INTERVAL}s ==="
last=""
while true; do
  nmcli device wifi rescan ifname "$IFACE" >/dev/null 2>&1
  if ssid_visible; then
    # 有局域网 -> 连/保持车 AP
    if ! is_active "$AP_CONN"; then
      [ "$last" != "join" ] && log "扫到车热点 '$AP_SSID' -> 连局域网"
      last="join"; join_ap
    elif ! peer_ok; then
      [ "$last" != "rejoin" ] && log "已连但 ping 不通车 $PEER_IP -> 重连局域网"
      last="rejoin"; join_ap
    else
      [ "$last" != "lan_ok" ] && log "局域网正常: 已连车 AP 且 ping 通 $PEER_IP"
      last="lan_ok"
    fi
  else
    # 无局域网 -> 连/保持默认上网 Wi-Fi
    if ! is_active "$UPLINK_CONN"; then
      [ "$last" != "uplink" ] && log "未扫到车热点 -> 连默认上网 Wi-Fi '$UPLINK_CONN'"
      last="uplink"; join_uplink
    else
      [ "$last" != "uplink_ok" ] && log "无车热点,已连上网 Wi-Fi '$UPLINK_CONN'"
      last="uplink_ok"
    fi
  fi
  sleep "$INTERVAL"
done
