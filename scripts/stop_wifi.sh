#!/usr/bin/env bash
# 停止飞车(STA)Wi-Fi 自动择网心跳。停止后 wlan0 保持当前所连(局域网或上网 Wi-Fi),不再自动切换。
# 想恢复自动择网:重新跑 ./autostart_wifi.sh(或重启,会话与启动会拉起)。
set -o pipefail
pkill -f autostart_wifi.sh && echo "已停心跳(autostart_wifi)" || echo "心跳未在跑"
