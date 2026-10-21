#!/usr/bin/env bash
# ============================================================
# PM01 一键启动（使用 ~/pm01_ws）
#
# 拉起:
#   0. .163 运控: 停 pm_v2 → 启动 pm01_edu
#      → idle → passive → pd_stand → walk → 校验 /cmd_vel
#   1. 相机 + YOLO          (~/pm01_ws/src/yolo_person_3d, require_lock=true)
#   2. 跟随控制 + 安全桥    armed=true（未锁定前感知不发目标，机器人不动）
#   3. 语音交互
#
# 锁定跟随:
#   启动后不会自动跟人。说“你好小智，跟着我”锁定正前方的人；
#   目标被挡住/走出画面会立即停下等待，不会改跟别人；
#   说“重新锁定”可换人，说“停止跟随”解除锁定。
#
# 用法:
#   ~/pm01_ws/start_all.sh              # 交互确认后启动
#   ~/pm01_ws/start_all.sh -y           # 跳过确认（后台/脚本调用时用）
#   ~/pm01_ws/start_all.sh --no-motion  # 跳过 .163 启动，只校验现有 walk
#   ~/pm01_ws/start_all.sh --motion-only # 只启动 .163 运控并切到 walk
#
# 非交互启动 .163 运控:
#   PM01_163_SUDO_PASSWORD='...' ~/pm01_ws/start_all.sh -y
#
# 日志: /tmp/pm01_follow_voice/{motion,vision,follow,voice}.log
#
# 急停:
#   1. 喊 "你好小智，停止跟随"
#   2. 本脚本 Ctrl+C（本机节点停止；.163 pm01_edu 保持运行）
#   3. 运控切回 pd_stand / 遥控器急停
# ============================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ASSUME_YES=0
MOTION_ENABLED=1
MOTION_ONLY=0

usage() {
    cat <<'EOF'
PM01 一键启动（使用 ~/pm01_ws）

拉起:
  0. .163 运控: 停 pm_v2 → 启动 pm01_edu
     → idle → passive → pd_stand → walk → 校验 /cmd_vel
  1. 相机 + YOLO          (~/pm01_ws/src/yolo_person_3d, require_lock=true)
  2. 跟随控制 + 安全桥    armed=true（未锁定前不发目标）
  3. 语音交互

锁定跟随:
  启动后不会自动跟人。说“你好小智，跟着我”锁定正前方的人；
  目标被挡住/走出画面会立即停下等待，不会改跟别人。

用法:
  ~/pm01_ws/start_all.sh              # 交互确认后启动
  ~/pm01_ws/start_all.sh -y           # 跳过确认（后台/脚本调用时用）
  ~/pm01_ws/start_all.sh --no-motion  # 跳过 .163 启动，只校验现有 walk
  ~/pm01_ws/start_all.sh --motion-only # 只启动 .163 运控并切到 walk

非交互启动 .163 运控:
  PM01_163_SUDO_PASSWORD='...' ~/pm01_ws/start_all.sh -y

日志: /tmp/pm01_follow_voice/{motion,vision,follow,voice}.log

急停:
  1. 喊 "你好小智，停止跟随"
  2. 本脚本 Ctrl+C（本机节点停止；.163 pm01_edu 保持运行）
  3. 运控切回 pd_stand / 遥控器急停
EOF
}

for arg in "$@"; do
    case "$arg" in
        -y|--yes)      ASSUME_YES=1 ;;
        --no-motion)   MOTION_ENABLED=0 ;;
        --motion-only) MOTION_ONLY=1 ;;
        -h|--help)     usage; exit 0 ;;
        *)
            printf '未知参数: %s\n' "$arg" >&2
            usage >&2
            exit 2
            ;;
    esac
done

# 只起运控时不需要语音 Key；完整链路必须在启动任何节点前检查。
if [ "$MOTION_ONLY" -ne 1 ] && [ -z "${DASHSCOPE_API_KEY:-}" ]; then
    echo ">> 错误: 未设置 DASHSCOPE_API_KEY（启动任何节点前退出）" >&2
    echo ">> 在已加载 ~/.bashrc 的交互终端运行，或先 export DASHSCOPE_API_KEY。" >&2
    exit 1
fi

source "$HERE/env.sh"

LOG_DIR=/tmp/pm01_follow_voice
mkdir -p "$LOG_DIR"
PIDS=()
MOTION_STAGE_STARTED=0
MOTION_READY=0

cleanup() {
    echo
    echo ">> 正在停止本机全部节点..."
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo ">> 本机节点已停止。"

    if [ "$MOTION_READY" -eq 1 ]; then
        echo ">> 注意: .163 的 pm01_edu 仍在运行，机器人仍处于 walk 控制。"
        echo ">> 让它站住不动:"
        echo "     source ~/pm01_ws/env.sh"
        echo "     ros2 topic pub --once /motion/set_motion_state \\"
        echo "       interface_protocol/msg/MotionStateRequest \"{target_motion_name: 'pd_stand'}\""
        echo ">> 彻底停止 pm01_edu（机器人需已趴下/有支撑）:"
        echo "     ssh user@192.168.0.163 'sudo pkill -f \"src_executor pm01_edu\"'"
        echo ">> 恢复原厂 pm_v2:"
        echo "     ssh user@192.168.0.163 'sudo systemctl start robotics.service'"
    elif [ "$MOTION_STAGE_STARTED" -eq 1 ]; then
        echo ">> 注意: .163 可能仍在启动/切换 pm01_edu。"
        echo ">> 先确认机器人姿态，再决定是否继续；查看:"
        echo "     ssh user@192.168.0.163 'tail -n 40 /tmp/engineai_pm01_edu.log'"
    else
        echo ">> 请把运控切回 pd_stand / passive。"
    fi
    exit 0
}
trap cleanup INT TERM

if [ "$ASSUME_YES" -ne 1 ]; then
    echo ">> 即将启动:"
    if [ "$MOTION_ENABLED" -eq 1 ]; then
        echo ">>   [0] .163 运控: 停 pm_v2 → 起 pm01_edu → idle→passive→pd_stand→walk"
    else
        echo ">>   [0] 跳过 .163 启动，仅校验现有 walk 运控"
    fi
    echo ">>   [1] 相机 + YOLO（严格锁定模式）"
    echo ">>   [2] 跟随控制 (armed=true，但未说“跟着我”不会发目标)"
    echo ">>   [3] 语音交互"
    echo ">> 请确认:"
    echo ">>   - 机器人已趴下/有支撑，周围清空，急停可用；"
    echo ">>   - passive 会松关节，pd_stand 会站起，walk 会进入行走控制；"
    echo ">>   - 人先离开机器人正前方；"
    echo ">>   - 启动后必须语音说“跟着我”才会开始跟随，目标丢失会停下等待。"
    read -r -p ">> 输入 yes 继续: " answer
    if [ "$answer" != "yes" ]; then
        echo ">> 已取消。"
        exit 1
    fi
fi

# ------------------------------------------------------------
# [0] .163 运控
# ------------------------------------------------------------

MOTION_STAGE_STARTED=1
if [ "$MOTION_ENABLED" -eq 1 ]; then
    echo ">> [0/3] .163 运控 (pm01_edu)... 日志: $LOG_DIR/motion.log"
    # start_all 的确认已经覆盖运控步骤，这里不再重复问 yes；
    # motion_up 仍会按需提示 .163 的 sudo 密码。
    if ! "$HERE/motion_up.sh" -y 2>&1 |
            tee "$LOG_DIR/motion.log"; then
        echo ">> 运控启动失败，未启动感知/跟随/语音。" >&2
        echo ">> 检查 .163: ssh user@192.168.0.163 'tail -n 40 /tmp/engineai_pm01_edu.log'" >&2
        exit 1
    fi
else
    echo ">> [0/3] 跳过 .163 启动，校验现有 walk 运控..."
    if ! "$HERE/motion_up.sh" --check 2>&1 |
            tee "$LOG_DIR/motion.log"; then
        echo ">> 现有运控不是 walk，或 /cmd_vel 没有 ros2_control_node 订阅者。" >&2
        echo ">> 先运行 ~/pm01_ws/motion_up.sh，或去掉 --no-motion。" >&2
        exit 1
    fi
fi
MOTION_READY=1

if [ "$MOTION_ONLY" -eq 1 ]; then
    echo ">> --motion-only: 运控已就绪，退出。"
    exit 0
fi

# ------------------------------------------------------------
# [1] 感知
# ------------------------------------------------------------

echo ">> [1/3] 感知(相机+YOLO, require_lock=true)... 日志: $LOG_DIR/vision.log"
ros2 launch yolo_person_3d bringup.launch.py require_lock:=true \
    > "$LOG_DIR/vision.log" 2>&1 &
PIDS+=($!)
sleep 8   # 等 TensorRT engine 加载

# ------------------------------------------------------------
# [2] 跟随
# ------------------------------------------------------------

echo ">> [2/3] 跟随控制(armed=true)... 日志: $LOG_DIR/follow.log"
ros2 launch pm01_aruco_detection person_follow.launch.py armed:=true \
    > "$LOG_DIR/follow.log" 2>&1 &
PIDS+=($!)
sleep 3

# ------------------------------------------------------------
# [3] 语音
# ------------------------------------------------------------

echo ">> [3/3] 语音交互... 日志: $LOG_DIR/voice.log"
"$HERE/run_voice.sh" > "$LOG_DIR/voice.log" 2>&1 &
PIDS+=($!)

echo ">> 全部启动完成。唤醒词: 你好小智"
echo ">> 锁定跟随: 说\"跟着我\"（未锁定前机器人不会跟人；目标丢失会停下等待）"
echo ">> 急停: 喊\"停止跟随\" 或 Ctrl+C"
echo ">> 日志: $LOG_DIR/{motion,vision,follow,voice}.log"

wait || true
cleanup
