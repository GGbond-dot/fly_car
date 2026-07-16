#!/usr/bin/env bash
# 飞车 carto 定位质量诊断 —— 一条命令搞定:清孤儿 → 只起 carto → 等 TF → 跑
# carto_quality_check.py（静止态采样,自出 ✅/❌ 结论),可连跑多轮。
#
# 为什么单独一个壳:carto_quality_check.py 自己不起 launch。而且这个测试**只起 carto**,
# 不起底盘/route_test_node —— 车绝不会动(不发 /cmd_vel),用来把 carto 从控制器里择干净:
# 静止的车如果 pose 都在抖/跳,就是图的问题,再调增益也白搭。
#
# 与 l_path_test.sh 一样按进程组起 launch,退出时 killpg 整组带走 + 兜底扫孤儿
# （手动 pkill 杀不掉子节点,会变孤儿,下次再起就重影)。
#
# 用法(飞车板;车放稳别推它,这测试不发速度,拆不拆桨都安全):
#   ./scripts/carto_quality_test.sh                 # 编一次 carto → 跑 1 轮
#   ./scripts/carto_quality_test.sh --runs 5        # carto 只起一次,连跑 5 轮看稳不稳
#   ./scripts/carto_quality_test.sh --no-build --runs 3
#   ./scripts/carto_quality_test.sh --duration-s 60 # 每轮采 60s(透传给 python)
#   ./scripts/carto_quality_test.sh --yaw-std-deg 0.2 --runs 3   # 收紧阈值
#
# 每轮产物: ~/kian_flycar/test_log/carto_quality/<时间戳>/(samples.csv + summary)。
# 注意:不开 set -u —— ROS 的 setup.bash 会引用未定义变量,开了直接报错退出。
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"   # 脚本在 <ws>/scripts/,ws 根 = 上一级

LOG_DIR="${PATROL_LOG_DIR:-$HOME/patrol_logs}"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/carto_quality_launch_$(date '+%m%d_%H%M%S').log"

DO_BUILD=true
RUNS=1
CHECK_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no-build) DO_BUILD=false ;;
    --runs)     RUNS="$2"; shift ;;
    --runs=*)   RUNS="${1#*=}" ;;
    -h|--help)  sed -n '2,24p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)          CHECK_ARGS+=("$1") ;;   # 其余(--duration-s / --yaw-std-deg …)透传给 python
  esac
  shift
done
case "$RUNS" in ''|*[!0-9]*) echo "!!! --runs 要正整数,收到 '$RUNS'"; exit 2 ;; esac

export ROS_DOMAIN_ID=1

# 杀孤儿:按可执行名杀(这测试只起 carto 那几个进程)。
kill_orphans() {
  for exe in cartographer_node bluesea2 robot_state_publisher; do
    pkill -f "$exe" 2>/dev/null
  done
}

LAUNCH_PGID=""
cleanup() {
  echo ""
  echo "--- 收 carto launch(整个进程组 + 兜底扫孤儿)..."
  [ -n "$LAUNCH_PGID" ] && kill -INT "-$LAUNCH_PGID" 2>/dev/null
  sleep 2
  [ -n "$LAUNCH_PGID" ] && kill -TERM "-$LAUNCH_PGID" 2>/dev/null
  kill_orphans
  sleep 1
}
trap cleanup EXIT INT TERM

echo "=== 飞车 carto 定位质量诊断: ws=$WS_ROOT  域=$ROS_DOMAIN_ID  轮数=$RUNS ==="
echo "    车放稳、别推它;这轮只起 carto,不发速度,车不会动。"
source /opt/ros/humble/setup.bash

if [ "$DO_BUILD" = true ]; then
  # 只装 my_carto_pkg —— fly_carto.launch.py 的启动时机改动要重装才生效。
  echo "--- 编/装 my_carto_pkg(让 carto 提前启动的改动生效)..."
  if ! (cd "$WS_ROOT" && colcon build --symlink-install \
        --packages-select my_carto_pkg 2>&1 | tail -8); then
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
  echo "--- 装好了"
fi

if [ ! -f "$WS_ROOT/install/setup.bash" ]; then
  echo "!!! 找不到 $WS_ROOT/install/setup.bash —— 这个工作空间没编过?"
  exit 1
fi
source "$WS_ROOT/install/setup.bash"

echo "--- 清掉可能残留的旧节点(孤儿)..."
kill_orphans
sleep 2

echo "--- 起 carto launch(独立进程组,日志 → $LOG)"
# setsid 让 launch 自成进程组 —— 退出时整组 killpg,子节点不会变孤儿。
setsid bash -c "source /opt/ros/humble/setup.bash; source '$WS_ROOT/install/setup.bash'; \
  export ROS_DOMAIN_ID=1; \
  exec ros2 launch my_carto_pkg fly_carto.launch.py" >"$LOG" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID=$LAUNCH_PID   # setsid 下 pgid == pid

# carto 在 launch 的 5s 定时器后起,起来后还要几秒扫描匹配才出连续 TF。
# 这里只盯 launch 活着、固定等 12s;TF 到没到交给 python 的 --warmup-s 自己判(它直查 TF,
# 不数 ros2 node —— daemon 缓存反复起停后全是幽灵节点,数节点会误报)。
echo "--- 等 carto 起齐(carto 在 5s 定时器后,固定等 12s)..."
for i in $(seq 1 12); do
  sleep 1
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "!!! carto launch 半路退出了。先死的是哪个(这才是根因):"
    echo "------------------------------------------------------------"
    grep -nE "process has died|exit code|not found|Caught exception|fatal|Traceback|No such|serial|open" "$LOG" | head -12
    echo "------------------------------------------------------------"
    echo "(完整日志: $LOG)"
    exit 1
  fi
done
echo "--- carto 起来了,开始跑诊断(python 会自己再等 TF 稳)"

overall_rc=0
declare -a VERDICTS
for r in $(seq 1 "$RUNS"); do
  echo ""
  echo "############## 第 $r/$RUNS 轮 ##############"
  echo "------------------------------------------------------------"
  python3 "$SCRIPT_DIR/carto_quality_check.py" "${CHECK_ARGS[@]}"
  rc=$?
  echo "------------------------------------------------------------"
  case "$rc" in
    0) VERDICTS+=("第$r轮: ✅ OK") ;;
    1) VERDICTS+=("第$r轮: ❌ 可疑"); overall_rc=1 ;;
    *) VERDICTS+=("第$r轮: ⚠ 出错(rc=$rc)"); overall_rc=1 ;;
  esac
done

echo ""
echo "============ 多轮汇总（$RUNS 轮）============"
for v in "${VERDICTS[@]}"; do echo "  $v"; done
echo "  产物目录: $HOME/kian_flycar/test_log/carto_quality/"
if [ "$overall_rc" = 0 ]; then
  echo "  → 每轮都 ✅:carto 静止定位没问题,慢/不准的锅不在图,回去调控制增益。"
else
  echo "  → 有 ❌/出错:静止车 pose 在抖或跳,先看 summary 里哪一项挂了。"
fi
exit "$overall_rc"
