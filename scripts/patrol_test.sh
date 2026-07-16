#!/usr/bin/env bash
# 陆空协同搜救 —— 飞车侧一键测试。**测这套东西只跑这一个脚本。**
# 车侧对应 car/scripts/patrol_test.sh(那边也是一条命令)。
#
# 做的事:
#   1) 注入 ROS_DOMAIN_ID=1(飞车在域1,与车板域0 隔离;跨机走裸 UDP 不受影响)
#      —— 手动敲 ros2 命令前也必须 export,否则啥都看不到
#   2) source ROS + 本工作空间
#   3) colcon build activity_control_pkg + my_launch
#      · activity_control_pkg = 桥(FC0D 到达检测/FC0E 放行/FC05 难民点)+ 航点执行器
#        (route_target_publisher 的话题订阅是新加的,没编就还是"只认启动参数")
#      · my_launch = launch 文件本身(靠 setup.py 装到 share/,不编跑的还是旧的)
#   4) 起 my_launch/patrol_ground.launch.py(carto/地面链/飞控链/航点/桥/投放全套),
#      日志刷屏太厉害,丢文件
#   5) 前台跑观察器:干净时间线 + **Ctrl-C 后直接打结论表**
#
# 跑法(飞车板):
#   ./scripts/patrol_test.sh                     # 正常测
#   ./scripts/patrol_test.sh --no-video          # 不起 YOLO(省 NPU,平板就没飞车画面)
#   ./scripts/patrol_test.sh --no-build          # 确定编过了,省时间
#   ./scripts/patrol_test.sh --no-rescuee        # 关掉难民点触发(坐标还没实测时用)
#
# 配合:车板跑 car/scripts/patrol_test.sh,然后在平板喊「开始搜救」。
#
# ⚠⚠ 第一次跑**别装桨** —— 两批下发/起飞两步/到达检测/投放中断都没上过板。
# ⚠  别跟旧剧本(relief_drop_ground.launch.py)同时起,会抢 /target_position。
#
# 注意:不要开 set -u。ROS 的 setup.bash 会引用未定义变量,开了直接报错退出。
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 脚本在 <fly_car>/scripts/,工作空间根 = 上一级(src/ 的父目录)
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LOG_DIR="${PATROL_LOG_DIR:-$HOME/patrol_logs}"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/patrol_ground_$(date '+%m%d_%H%M%S').log"

DO_BUILD=true
LAUNCH_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --no-build)   DO_BUILD=false ;;
    --no-video)   LAUNCH_ARGS+=("--launch-arg" "with_video:=false") ;;
    --no-rescuee) LAUNCH_ARGS+=("--launch-arg" "rescuee_check_hz:=0.0") ;;
    -h|--help)    sed -n '2,27p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)            LAUNCH_ARGS+=("$arg") ;;
  esac
done

# 飞车整条链路进域 1。**必须在起任何节点之前 export** —— 观察器和 launch 都要在同一个域。
export ROS_DOMAIN_ID=1

echo "=== 飞车侧巡航/投放测试: ws=$WS_ROOT  域=$ROS_DOMAIN_ID ==="
source /opt/ros/humble/setup.bash

if [ "$DO_BUILD" = true ]; then
  echo "--- 编 activity_control_pkg + my_launch ..."
  # ⚠ 别把 colcon 输出 tail 掉 —— 真正的编译错误在 log/latest_build/<pkg>/stderr.log,
  #   摘要只会说"1 package failed"(07-16 踩的)。
  if ! (cd "$WS_ROOT" && colcon build --symlink-install \
        --packages-select activity_control_pkg my_launch 2>&1 | tail -8); then
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

# launch 里的 ExecuteProcess 按板上绝对路径找脚本,少一个就静默失败(摄像头不切/投放没反应)
for s in servo_camera_by_mode.py rescue_drop_sequencer.py; do
  [ -f "$HOME/kian_flycar/scripts/$s" ] || \
    echo "!!! 警告: ~/kian_flycar/scripts/$s 不在 —— launch 会静默跳过它"
done

echo "--- launch 日志: $LOG"
echo "--- 起节点 + 观察器。看完按 Ctrl-C,结论表会自己打出来。"
echo "--- ⚠ 第一次跑别装桨。"
echo

# 观察器负责拉起 launch 并在退出时连同子节点一起收干净(整个进程组)
exec python3 "$SCRIPT_DIR/patrol_monitor_fly.py" --launch --log "$LOG" "${LAUNCH_ARGS[@]}"
