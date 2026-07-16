#!/usr/bin/env bash
# 飞车固定 L 直线调参 —— 一条命令搞定:清孤儿 → 起 launch → 等节点齐 → 跑 l_path_tuning。
#
# 为什么要这个壳:l_path_tuning.py 自己不起 launch,得另开终端手动 `ros2 launch`。
# 而手动起的 launch 用 pkill 杀不干净 —— route_test_node/diff_drive_controller 这些
# 子节点的命令行里没有 "patrol_ground" 字样,pkill -f patrol_ground 只杀启动器,
# 子节点变孤儿活着,下次再起就出现两个 route_test_node,参数操作全乱(07-16 踩的)。
# 本脚本按进程组起 launch,退出时 killpg 整组带走,再兜底扫一遍孤儿。
#
# ⚠⚠ 调参要车真跑,**必须拆掉全部桨**。
#
# 用法(飞车板,想调哪个参数就 --set 哪个,可多个;不带 --set 就用脚本内基准):
#   ./scripts/l_path_test.sh --dry-run                 # 只自检,不设参数不发车
#   ./scripts/l_path_test.sh                           # 用基准参数跑一轮
#   ./scripts/l_path_test.sh --set straight_kp_w=0.18  # 只改一个,其余走基准
#   ./scripts/l_path_test.sh --no-build --set straight_kp_w=0.14 --set straight_w_max_rps=0.12
#   ./scripts/l_path_test.sh --keep-launch --set ...   # 跑完不关 launch,连着调下一轮更快
#
# 每轮产物在 ~/kian_flycar/test_log/l_path_runs/<时间戳>/(samples.csv + params.yaml + summary)。
#
# 注意:不开 set -u。ROS 的 setup.bash 会引用未定义变量,开了直接报错退出。
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"   # 脚本在 <ws>/scripts/,ws 根 = 上一级

LOG_DIR="${PATROL_LOG_DIR:-$HOME/patrol_logs}"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/l_path_launch_$(date '+%m%d_%H%M%S').log"

DO_BUILD=true
KEEP_LAUNCH=false
TUNE_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --no-build)    DO_BUILD=false ;;
    --keep-launch) KEEP_LAUNCH=true ;;
    -h|--help)     sed -n '2,25p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)             TUNE_ARGS+=("$arg") ;;   # 其余(--dry-run / --set K=V / --timeout-s …)透传给 tuning
  esac
done

export ROS_DOMAIN_ID=1

# 杀孤儿:按**可执行名**杀,不是按 launch 名 —— 这正是手动 pkill 杀不掉的那批。
kill_orphans() {
  for exe in patrol_ground route_test_node diff_drive_controller chassis_mux \
             cartographer_node xmachine_bridge position_pid uart_to_stm32 \
             bluesea2 robot_state_publisher; do
    pkill -f "$exe" 2>/dev/null
  done
}

LAUNCH_PGID=""
cleanup() {
  if [ "$KEEP_LAUNCH" = true ] && [ -n "$LAUNCH_PGID" ]; then
    echo ""
    echo "--- --keep-launch:launch 还开着(pgid=$LAUNCH_PGID)。接着调下一轮:"
    echo "      ./scripts/l_path_test.sh --no-build --keep-launch --set 参数=值"
    echo "    调完手动收干净: kill -TERM -$LAUNCH_PGID ; $0 的 pkill 那几行"
    return
  fi
  echo ""
  echo "--- 收 launch(整个进程组 + 兜底扫孤儿)..."
  [ -n "$LAUNCH_PGID" ] && kill -INT "-$LAUNCH_PGID" 2>/dev/null
  sleep 2
  [ -n "$LAUNCH_PGID" ] && kill -TERM "-$LAUNCH_PGID" 2>/dev/null
  kill_orphans
  sleep 1
}
trap cleanup EXIT INT TERM

echo "=== 飞车固定 L 调参: ws=$WS_ROOT  域=$ROS_DOMAIN_ID ==="
echo "⚠⚠ 拆桨了吗?这一轮车会真跑。"
source /opt/ros/humble/setup.bash

if [ "$DO_BUILD" = true ]; then
  echo "--- 编 ground_chassis_pkg + my_launch ..."
  if ! (cd "$WS_ROOT" && colcon build --symlink-install \
        --packages-select ground_chassis_pkg my_launch 2>&1 | tail -8); then
    echo ""
    echo "!!! colcon build 失败。真正的错误(log/latest_build/*/stderr.log):"
    echo "------------------------------------------------------------"
    for f in "$WS_ROOT"/log/latest_build/*/stderr.log; do
      [ -s "$f" ] || continue
      echo "### $(basename "$(dirname "$f")")"
      tail -40 "$f"
    done
    echo "------------------------------------------------------------"
    exit 1
  fi
  echo "--- 编好了"
fi

if [ ! -f "$WS_ROOT/install/setup.bash" ]; then
  echo "!!! 找不到 $WS_ROOT/install/setup.bash —— 这个工作空间没编过?"
  exit 1
fi
source "$WS_ROOT/install/setup.bash"

# 起 launch 前先扫掉可能残留的孤儿,保证只有这一个 launch。
echo "--- 清掉可能残留的旧节点(孤儿)..."
kill_orphans
sleep 2

echo "--- 起 launch(独立进程组,日志 → $LOG)"
# setsid 让 launch 自成进程组 —— 退出时能整组 killpg,子节点不会变孤儿。
# 用**最小** launch(l_path_tune):只起 carto + 地面底盘 + route_test_node。
# 不用 patrol_ground —— 那个还起 servo_camera(也开 /dev/ttyS3,跟底盘抢串口)、飞控、
# 投放、yolo,全跟磨直线无关,而且串口冲突/yolo 崩会把整个 launch 带下水(07-16 踩的)。
setsid bash -c "source /opt/ros/humble/setup.bash; source '$WS_ROOT/install/setup.bash'; \
  export ROS_DOMAIN_ID=1; \
  exec ros2 launch my_launch l_path_tune.launch.py" >"$LOG" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID=$LAUNCH_PID   # setsid 下 pgid == pid

# 运动链挂在 launch 的 12s 定时器后(等 carto 出 TF)。这里**只**盯 launch 进程活着,
# 固定等 20s 让节点起齐 —— **绝不用 ros2 node list 数节点**:它读的是 ros2 daemon 缓存,
# 反复起停后缓存里全是死节点的幽灵,数出来"2 个 route_test_node"就误报中止(07-16 栽了
# 一整晚)。节点齐没齐交给 l_path_tuning.py 自己判 —— 它用直接 DDS 发现(不走 daemon),
# 检查 required 节点/TF/flight_enable,不齐会干净报错。
echo "--- 等 launch 起齐(运动链在 12s 定时器后,固定等 20s)..."
for i in $(seq 1 20); do
  sleep 1
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "!!! launch 半路退出了。哪个节点先死的(这才是根因,不是后面的收尾):"
    echo "------------------------------------------------------------"
    grep -nE "process has died|exit code|not found|Caught exception|required|fatal|Traceback|No such" "$LOG" | head -10
    echo "------------------------------------------------------------"
    echo "(完整日志: $LOG)"
    exit 1
  fi
done
echo "--- launch 起来了,交给 l_path_tuning 自检(它用 DDS 直查,不看 daemon 缓存)"

echo "--- 跑 l_path_tuning.py ${TUNE_ARGS[*]}"
echo "------------------------------------------------------------"
python3 "$SCRIPT_DIR/l_path_tuning.py" "${TUNE_ARGS[@]}"
rc=$?
echo "------------------------------------------------------------"
if [ "$rc" = 0 ]; then
  echo "--- 完成。产物: $HOME/kian_flycar/test_log/l_path_runs/<最新时间戳>/"
  echo "    把整个时间戳目录拉回来(samples.csv + params.yaml 都要),别只发 summary。"
else
  echo "!!! l_path_tuning 退出码 $rc —— 上面有原因"
fi
exit $rc
