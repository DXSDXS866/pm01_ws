#!/usr/bin/env bash
# ============================================================
# PM01 运控切换：.163 上的 pm01_edu / pm_v2
#
# 用法:
#   ./motion_163.sh status        # 查看当前运控/状态（不需要 sudo 密码）
#   ./motion_163.sh start         # 停 robotics.service，启动 pm01_edu
#   ./motion_163.sh switch-walk   # idle -> passive -> pd_stand -> walk
#   ./motion_163.sh stop          # 停 pm01_edu（机器人可能失去支撑）
#   ./motion_163.sh restore       # 停 pm01_edu，恢复原厂 pm_v2 服务
#
# 可选环境变量:
#   PM01_MOTION_HOST=user@192.168.0.163
#   PM01_163_SUDO_PASS=<.163 sudo 密码>   # 非交互运行必须设置
#   PM01_MOTION_DIR=/home/user/projects/engineai_robotics
#
# 安全:
#   start / switch-walk 会让机器人从趴姿站起并进入行走态。
#   执行前确认机器人周围无人、有急停、地面平整。
# ============================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_HOST="${PM01_MOTION_HOST:-user@192.168.0.163}"
ROBOT_DIR="${PM01_MOTION_DIR:-/home/user/projects/engineai_robotics}"
UNIT="${PM01_MOTION_UNIT:-pm01-edu}"
REMOTE_LOG="${PM01_MOTION_LOG:-/tmp/pm01_edu.log}"

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=5)
ASSUME_YES=0
ARGS=()

for arg in "$@"; do
    case "$arg" in
        -y|--yes) ASSUME_YES=1 ;;
        -h|--help) ARGS+=("--help") ;;
        *) ARGS+=("$arg") ;;
    esac
done
set -- "${ARGS[@]}"

usage() {
    cat <<'EOF'
用法:
  ./motion_163.sh status        # 查看当前运控/状态（不需要 sudo 密码）
  ./motion_163.sh start         # 停 robotics.service，启动 pm01_edu
  ./motion_163.sh switch-walk   # idle -> passive -> pd_stand -> walk
  ./motion_163.sh stop          # 停 pm01_edu（机器人可能失去支撑）
  ./motion_163.sh restore       # 停 pm01_edu，恢复原厂 pm_v2 服务

选项:
  -y, --yes                     跳过交互确认（脚本/后台调用）

环境变量:
  PM01_MOTION_HOST              默认 user@192.168.0.163
  PM01_163_SUDO_PASS            .163 的 sudo 密码（非交互运行必须设置）
  PM01_MOTION_DIR               默认 /home/user/projects/engineai_robotics
  PM01_MOTION_UNIT              默认 pm01-edu
  PM01_MOTION_LOG               默认 /tmp/pm01_edu.log
EOF
}

need_pass() {
    if [ -n "${PM01_163_SUDO_PASS:-}" ]; then
        return 0
    fi
    if [ -t 0 ]; then
        read -rsp ">> $ROBOT_HOST sudo 密码: " PM01_163_SUDO_PASS
        echo
        export PM01_163_SUDO_PASS
    else
        echo ">> 缺少 PM01_163_SUDO_PASS（$ROBOT_HOST 的 sudo 密码）" >&2
        return 1
    fi
}

ssh_run() {
    ssh "${SSH_OPTS[@]}" "$ROBOT_HOST" "$@"
}

sudo_run() {
    local cmd="$1"
    need_pass
    # 密码走 stdin，不出现在 argv/ps 里。
    printf '%s\n' "$PM01_163_SUDO_PASS" |
        ssh "${SSH_OPTS[@]}" "$ROBOT_HOST" \
            "sudo -S -p '' bash -c $(printf '%q' "$cmd")"
}

ros_run() {
    local cmd="$1"
    ssh_run "source ~/.bashrc >/dev/null 2>&1; \
             source /opt/ros/humble/setup.bash >/dev/null 2>&1; $cmd"
}

get_state() {
    ros_run "timeout 3 ros2 topic echo /motion/motion_state \
             --qos-reliability best_effort --once 2>/dev/null" 2>/dev/null |
        awk '/current_motion_task:/ {print $2; exit}'
}

wait_state() {
    local want="$1" timeout_s="${2:-20}" i state=""
    for ((i = 1; i <= timeout_s; i++)); do
        state="$(get_state || true)"
        if [ "$state" = "$want" ]; then
            echo ">> 运动状态: $state"
            return 0
        fi
        sleep 1
    done
    echo ">> 等待 $want 超时（当前: ${state:-未知}）" >&2
    return 1
}

set_state() {
    local target="$1"
    ros_run "ros2 topic pub --once /motion/set_motion_state \
             interface_protocol/msg/MotionStateRequest \
             \"{target_motion_name: '$target'}\"" >/dev/null
}

confirm() {
    if [ "$ASSUME_YES" -eq 1 ]; then
        return 0
    fi
    local answer
    read -r -p ">> $1 输入 yes 继续: " answer
    [ "$answer" = "yes" ]
}

unit_active() {
    ssh_run "systemctl is-active --quiet '$UNIT'" 2>/dev/null
}

service_active() {
    ssh_run "systemctl is-active --quiet robotics.service" 2>/dev/null
}

cmd_status() {
    echo ">> .163: $ROBOT_HOST"
    echo -n ">> robotics.service (pm_v2): "
    service_active && echo active || echo inactive
    echo -n ">> $UNIT (pm01_edu):      "
    unit_active && echo active || echo inactive
    echo ">> 进程:"
    ssh_run "pgrep -af 'src_executor|run_robot' || true" | sed 's/^/   /'
    echo ">> /motion/motion_state:"
    echo "   current_motion_task: $(get_state || echo '无发布者')"
    echo ">> /cmd_vel 订阅:"
    ros_run "timeout 5 ros2 topic info /cmd_vel -v 2>/dev/null \
             | grep -E 'Subscription count|Node name' || true" | sed 's/^/   /'
}

cmd_start() {
    confirm "停止当前运控可能让机器人失去支撑。确认机器人已趴稳/有支撑、周围无人。" || {
        echo ">> 已取消"
        return 1
    }

    echo ">> 停止 robotics.service（pm_v2）..."
    sudo_run "systemctl stop robotics.service"

    local i
    for ((i = 1; i <= 20; i++)); do
        ssh_run "pgrep -f 'src_executor pm_v2' >/dev/null" 2>/dev/null || break
        sleep 1
    done
    if ssh_run "pgrep -f 'src_executor pm_v2' >/dev/null" 2>/dev/null; then
        echo ">> src_executor pm_v2 仍在运行，停止失败" >&2
        return 1
    fi

    if ssh_run "pgrep -f motor_service_server >/dev/null" 2>/dev/null; then
        echo ">> motor_service_server 正在运行；run_robot.sh 会拒绝启动。" >&2
        echo ">> 请先关闭机器人管理网页/电机服务。" >&2
        return 1
    fi

    if unit_active; then
        echo ">> $UNIT 已在运行，跳过启动"
    else
        sudo_run "systemctl reset-failed '$UNIT' 2>/dev/null || true
systemd-run --unit='$UNIT' --collect \
  --working-directory='$ROBOT_DIR' \
  --setenv=ROS_DOMAIN_ID=69 \
  --setenv=ROS_LOCALHOST_ONLY=0 \
  --setenv=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -p StandardOutput=append:$REMOTE_LOG \
  -p StandardError=append:$REMOTE_LOG \
  /bin/bash -lc './run_robot.sh pm01_edu'"
        echo ">> 已启动 $UNIT，日志: $REMOTE_LOG"
    fi

    echo ">> 等待 /motion/motion_state（最长 120s）..."
    local state=""
    for ((i = 1; i <= 120; i++)); do
        state="$(get_state || true)"
        if [ -n "$state" ]; then
            echo ">> pm01_edu 已就绪，当前状态: $state"
            return 0
        fi
        sleep 1
    done
    echo ">> 120s 内没有收到 /motion/motion_state；查看 $REMOTE_LOG" >&2
    return 1
}

cmd_switch_walk() {
    local state
    state="$(get_state || true)"
    echo ">> 当前运动状态: ${state:-未知}"

    if [ "$state" = "walk" ]; then
        echo ">> 已经是 walk，跳过"
        return 0
    fi

    confirm "机器人将从趴姿进入 passive → pd_stand → walk。确认周围无人、急停可用。" || {
        echo ">> 已取消"
        return 1
    }

    echo ">> → passive"
    set_state passive
    wait_state passive 8 || true

    echo ">> → pd_stand（站起，最长等 25s）"
    set_state pd_stand
    if ! wait_state pd_stand 25; then
        echo ">> 未进入 pd_stand，停止后续切换（不会自动进 walk）" >&2
        return 1
    fi

    echo ">> → walk"
    set_state walk
    if ! wait_state walk 15; then
        echo ">> 未进入 walk；可查看 $REMOTE_LOG" >&2
        return 1
    fi

    echo ">> 已进入 walk"
}

cmd_stop() {
    confirm "停止 pm01_edu 后机器人可能失去支撑。确认周围安全/已扶稳。" || {
        echo ">> 已取消"
        return 1
    }
    sudo_run "systemctl stop '$UNIT'" || true
    echo ">> $UNIT 已停止。恢复原厂 pm_v2: $0 restore"
}

cmd_restore() {
    confirm "将停止 pm01_edu 并恢复 pm_v2；切换过程中机器人可能失去支撑。" || {
        echo ">> 已取消"
        return 1
    }
    echo ">> 停止 $UNIT，恢复 robotics.service（pm_v2）..."
    sudo_run "systemctl stop '$UNIT' 2>/dev/null || true
systemctl start robotics.service"
    sleep 2
    echo -n ">> robotics.service: "
    service_active && echo active || echo inactive
}

case "${1:-}" in
    status) cmd_status ;;
    start) cmd_start ;;
    switch-walk|walk) cmd_switch_walk ;;
    stop) cmd_stop ;;
    restore) cmd_restore ;;
    ""|-h|--help) usage ;;
    *)
        echo ">> 未知命令: $1" >&2
        usage
        exit 2
        ;;
esac
