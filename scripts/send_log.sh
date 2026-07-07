#!/usr/bin/env bash
# send_log.sh — 在【开发板】上运行,把 test_log/ 的调参 CSV 回传到本地电脑。
#
# 用法(板上 ~/kian_flycar 下):
#   scripts/send_log.sh            # 只传最新的一个 csv(默认)
#   scripts/send_log.sh -n 3       # 传最新的 3 个
#   scripts/send_log.sh --all      # 传全部 csv
#
# 目标电脑固定为 kian@192.168.10.116:~/kian_flycar/test_log/(可用环境变量覆盖:
#   DEST_USER / DEST_HOST / DEST_DIR / SSH_PORT)。
# 首次会提示输入电脑登录密码(想免密:先在板上 ssh-copy-id kian@192.168.10.116)。
set -o pipefail

# ---- 目标电脑(可被环境变量覆盖)----
DEST_USER="${DEST_USER:-kian}"
DEST_HOST="${DEST_HOST:-192.168.10.116}"
DEST_DIR="${DEST_DIR:-~/kian_flycar/test_log}"
SSH_PORT="${SSH_PORT:-22}"

# ---- 路径解析:脚本在 <ws>/scripts/,test_log 在 <ws>/test_log ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$WS_DIR/test_log"

# ---- 参数:默认最新 1 个 ----
COUNT=1
SEND_ALL=0
case "$1" in
  --all) SEND_ALL=1 ;;
  -n)    COUNT="${2:-1}" ;;
  "")    ;;
  *)     echo "未知参数: $1(用 -n N 或 --all)"; exit 2 ;;
esac

if [ ! -d "$LOG_DIR" ]; then
  echo "找不到日志目录: $LOG_DIR"; exit 1
fi

# 按修改时间新->旧列出 csv
mapfile -t ALL_CSV < <(ls -1t "$LOG_DIR"/*.csv 2>/dev/null)
if [ "${#ALL_CSV[@]}" -eq 0 ]; then
  echo "$LOG_DIR 里没有 .csv 文件(先跑一次测试生成日志)"; exit 1
fi

if [ "$SEND_ALL" -eq 1 ]; then
  FILES=("${ALL_CSV[@]}")
else
  FILES=("${ALL_CSV[@]:0:$COUNT}")
fi

echo "将回传 ${#FILES[@]} 个文件到 ${DEST_USER}@${DEST_HOST}:${DEST_DIR}"
for f in "${FILES[@]}"; do echo "  - $(basename "$f")"; done

# 确保对端目录存在(scp 不会自动建父目录)
ssh -p "$SSH_PORT" "${DEST_USER}@${DEST_HOST}" "mkdir -p ${DEST_DIR}" || {
  echo "无法连到电脑或建目录失败,检查网络/IP/SSH。"; exit 1
}

scp -P "$SSH_PORT" "${FILES[@]}" "${DEST_USER}@${DEST_HOST}:${DEST_DIR}/" && {
  echo "完成。电脑上查看:  ls -t ${DEST_DIR}"
}
