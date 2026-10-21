#!/usr/bin/env bash
# ============================================================
# .163 运控栈 pm01_edu 启动 + 状态切换
#
# 做四件事:
#   1. SSH 到 .163，停掉原厂 robotics.service (pm_v2)
#   2. 启动 /home/user/projects/engineai_robotics/run_robot.sh pm01_edu
#   3. 等 pm01_edu 的 ROS2 桥就绪，按 idle → passive → pd_stand → walk 切换
#   4. 确认 /cmd_vel 有 ros2_control_node 订阅者（跟随链路才有效）
#
# 注意: pm01_edu 不发布 /motion/motion_state。它的状态从 .163 日志里的
#       "Entered motion [ xxx ]" 读取；启动后默认 idle（不打印切换行）。
#       只有没在跑 pm01_edu 时，才回退去读 pm_v2 的 /motion/motion_state。
#
# 用法:
#   ~/pm01_ws/motion_up.sh                 # 交互确认 + 输入 .163 sudo 密码
#   ~/pm01_ws/motion_up.sh -y              # 跳过运控确认（仍可能提示 sudo 密码）
#   ~/pm01_ws/motion_up.sh --check         # 只检查当前是否 walk + /cmd_vel 订阅者
#   ~/pm01_ws/motion_up.sh --dry-run       # 只打印将执行的步骤，不连 .163
#
# 非交互（例如 nohup/CI）:
#   PM01_163_SUDO_PASSWORD='...' ~/pm01_ws/motion_up.sh -y
#
# 可覆盖的环境变量:
#   PM01_163_HOST         默认 user@192.168.0.163
#   PM01_163_SDK_DIR      默认 /home/user/projects/engineai_robotics
#   PM01_163_PROFILE      默认 pm01_edu
#   PM01_163_LOG          默认 /tmp/engineai_pm01_edu.log（.163 上）
# ============================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh" >/dev/null

MOTION_HOST="${PM01_163_HOST:-user@192.168.0.163}"
MOTION_SDK_DIR="${PM01_163_SDK_DIR:-/home/user/projects/engineai_robotics}"
MOTION_PROFILE="${PM01_163_PROFILE:-pm01_edu}"
MOTION_LOG="${PM01_163_LOG:-/tmp/engineai_${MOTION_PROFILE}.log}"

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=5)

ASSUME_YES=0
CHECK_ONLY=0
DRY_RUN=0
SDK_TOUCHED=0
SUDO_PW=""

log()  { printf '>> %s\n' "$*"; }
warn() { printf '>> 警告: %s\n' "$*" >&2; }

usage() {
    cat <<'EOF'
.163 运控栈 pm01_edu 启动 + 状态切换

做四件事:
  1. SSH 到 .163，停掉原厂 robotics.service (pm_v2)
  2. 启动 /home/user/projects/engineai_robotics/run_robot.sh pm01_edu
  3. 等 pm01_edu 的 ROS2 桥就绪，按 idle → passive → pd_stand → walk 切换
  4. 确认 /cmd_vel 有 ros2_control_node 订阅者（跟随链路才有效）

状态来源: pm01_edu 不发布 /motion/motion_state，本脚本读 .163 日志里的
"Entered motion [ xxx ]"；启动后没有切换记录时按 idle 处理。

用法:
  ~/pm01_ws/motion_up.sh                 # 交互确认 + 输入 .163 sudo 密码
  ~/pm01_ws/motion_up.sh -y              # 跳过运控确认（仍可能提示 sudo 密码）
  ~/pm01_ws/motion_up.sh --check         # 只检查当前是否 walk + /cmd_vel 订阅者
  ~/pm01_ws/motion_up.sh --dry-run       # 只打印将执行的步骤，不连 .163

非交互（例如 nohup/CI）:
  PM01_163_SUDO_PASSWORD='...' ~/pm01_ws/motion_up.sh -y

可覆盖的环境变量:
  PM01_163_HOST         默认 user@192.168.0.163
  PM01_163_SDK_DIR      默认 /home/user/projects/engineai_robotics
  PM01_163_PROFILE      默认 pm01_edu
  PM01_163_LOG          默认 /tmp/engineai_pm01_edu.log（.163 上）
EOF
}

show_remote_log() {
    if [ "$SDK_TOUCHED" -eq 1 ] && [ "$DRY_RUN" -eq 0 ]; then
        printf '>> .163 %s 最后 40 行:\n' "$MOTION_LOG" >&2
        ssh "${SSH_OPTS[@]}" "$MOTION_HOST" \
            "tail -n 40 '$MOTION_LOG' 2>/dev/null" >&2 || true
    fi
}

die() {
    printf '>> 错误: %s\n' "$*" >&2
    show_remote_log
    exit 1
}

# ------------------------------------------------------------
# 读取 / 发布运动状态
#
# pm01_edu 不发布 /motion/motion_state，只在状态切换时往日志写
# "Entered motion [ xxx ]"。启动后直接处于 idle，不写这一行。
# ------------------------------------------------------------

remote_bash() {
    # 通过 stdin 把一段 bash 送到 .163 执行，stdout 原样返回
    printf '%s\n' "$1" | ssh -T "${SSH_OPTS[@]}" "$MOTION_HOST" 'bash -s' 2>/dev/null
}

remote_motion_probe() {
    # 输出:
    #   RUNNING <last_entered>   pm01_edu 在跑（没有切换记录时 <last_entered> 为空）
    #   STOPPED                  pm01_edu 没在跑
    #   UNKNOWN                  进程在跑，但日志缺失或不是这个进程写的
    remote_bash "
        pid=\$(pgrep -f '[s]rc_executor.*$MOTION_PROFILE' | head -n 1)
        if [ -n \"\$pid\" ]; then
            if [ ! -f '$MOTION_LOG' ]; then
                echo UNKNOWN
                exit 0
            fi
            now=\$(date +%s)
            lm=\$(stat -c %Y '$MOTION_LOG' 2>/dev/null || echo 0)
            et=\$(ps -o etimes= -p \"\$pid\" 2>/dev/null | tr -d ' ')
            # 日志最后一次写入早于进程启动 = 不是这次运行写的
            if [ -n \"\$et\" ] && [ \$((now - lm)) -gt \$((et + 5)) ]; then
                echo UNKNOWN
                exit 0
            fi
            s=\$(sed -n 's/.*Entered motion \\[ \\([A-Za-z0-9_]*\\) \\].*/\\1/p' '$MOTION_LOG' 2>/dev/null | tail -n 1)
            echo \"RUNNING \$s\"
        else
            echo STOPPED
        fi
    " || echo UNKNOWN
}

read_motion_state() {
    local probe state out

    probe="$(remote_motion_probe || true)"
    case "$probe" in
        RUNNING*)
            state="${probe#RUNNING}"
            state="${state# }"
            # 启动后没有切换记录 = idle
            printf '%s\n' "${state:-idle}"
            ;;
        STOPPED)
            # 没有 pm01_edu 时回退到原厂 pm_v2 的 /motion/motion_state
            out="$(timeout 4 ros2 topic echo /motion/motion_state \
                --qos-reliability best_effort --once 2>/dev/null || true)"
            printf '%s\n' "$out" |
                awk '/^current_motion_task:/ {print $2; exit}'
            ;;
        *)
            # UNKNOWN: 日志不可信，宁可不输出也不要猜状态
            ;;
    esac
}

print_motion_info() {
    local probe

    probe="$(remote_motion_probe || true)"
    case "$probe" in
        RUNNING*)
            printf '>> 当前状态: %s（来源: .163 日志）\n' "$(read_motion_state)"
            printf '>> %s 最近的切换记录:\n' "$MOTION_LOG"
            remote_bash "grep -E 'Entered motion|Exited motion|Manual Transition|Rejecting|transitioner' '$MOTION_LOG' | tail -n 15" || true
            ;;
        STOPPED)
            timeout 4 ros2 topic echo /motion/motion_state \
                --qos-reliability best_effort --once 2>/dev/null || true
            ;;
        *)
            printf '>> 无法从 %s 读取状态（日志缺失，或不是当前进程写的）\n' "$MOTION_LOG"
            remote_bash "pgrep -af '[s]rc_executor|[r]un_robot.sh' ; ls -l '$MOTION_LOG' 2>&1" || true
            ;;
    esac
}

publish_motion_state() {
    local target="$1"
    timeout 10 ros2 topic pub --once /motion/set_motion_state \
        interface_protocol/msg/MotionStateRequest \
        "{target_motion_name: '$target'}" >/dev/null
}

wait_for_state() {
    local target="$1" timeout_s="$2"
    local deadline=$((SECONDS + timeout_s))
    local cur

    while [ "$SECONDS" -lt "$deadline" ]; do
        cur="$(read_motion_state)"
        if [ "$cur" = "$target" ]; then
            return 0
        fi
        sleep 1
    done
    return 1
}

step_to() {
    local target="$1" timeout_s="$2"

    log "请求切换 → $target"
    if ! publish_motion_state "$target"; then
        die "发布 $target 请求失败"
    fi

    # 按现场流程至少等 2 秒，再等状态确认
    sleep 2
    if ! wait_for_state "$target" "$timeout_s"; then
        warn "切换到 $target 超时，当前状态信息:"
        print_motion_info >&2 || true
        die "未进入 $target"
    fi

    log "✓ 已进入 $target"
}

wait_cmd_vel_subscriber() {
    local timeout_s="${1:-20}"
    local deadline=$((SECONDS + timeout_s))
    local info count

    while [ "$SECONDS" -lt "$deadline" ]; do
        info="$(timeout 4 ros2 topic info /cmd_vel -v --no-daemon 2>/dev/null || true)"
        count="$(printf '%s\n' "$info" |
            awk '/Subscription count:/ {print $3; exit}')"
        if [ "${count:-0}" -ge 1 ] &&
           printf '%s\n' "$info" | grep -q 'ros2_control_node'; then
            return 0
        fi
        sleep 1
    done
    return 1
}

wait_motion_cmd_subscriber() {
    local timeout_s="${1:-20}"
    local deadline=$((SECONDS + timeout_s))
    local info

    while [ "$SECONDS" -lt "$deadline" ]; do
        info="$(timeout 4 ros2 topic info /motion/set_motion_state -v --no-daemon 2>/dev/null || true)"
        if printf '%s\n' "$info" | grep -q 'ros2_control_node'; then
            return 0
        fi
        sleep 1
    done
    return 1
}

# ------------------------------------------------------------
# .163 远端启动
# ------------------------------------------------------------

build_remote_script() {
    cat <<REMOTE
set -eo pipefail
trap 'rm -f "\$0"' EXIT

# 第一行 stdin 是 sudo 密码；用完立即从变量清掉，不写入文件
IFS= read -r SUDO_PW
printf '%s\n' "\$SUDO_PW" | sudo -S -p '' -v
unset SUDO_PW

echo ">> [.163] 停止 robotics.service (pm_v2)"
sudo -n systemctl stop robotics.service

# 等旧 src_executor 退干净（最多 5 秒）
for _ in 1 2 3 4 5; do
    pgrep -f '[s]rc_executor' >/dev/null || break
    sleep 1
done

if pgrep -f '[s]rc_executor' >/dev/null; then
    echo "ERROR: 仍有 src_executor 在运行，拒绝启动第二个" >&2
    pgrep -af '[s]rc_executor' >&2 || true
    exit 1
fi

cd '$MOTION_SDK_DIR'
echo ">> [.163] 启动 $MOTION_PROFILE"
sudo -n env ROS_DOMAIN_ID=69 ROS_LOCALHOST_ONLY=0 \\
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \\
    setsid nohup ./run_robot.sh '$MOTION_PROFILE' \\
    > '$MOTION_LOG' 2>&1 < /dev/null &
disown || true

# run_robot.sh 可能先做最长 60 秒的路由检查，这里只确认它没立刻退出
sleep 3
if ! pgrep -f '[r]un_robot.sh $MOTION_PROFILE' >/dev/null; then
    echo "ERROR: run_robot.sh $MOTION_PROFILE 没有起来" >&2
    tail -n 40 '$MOTION_LOG' >&2 || true
    exit 1
fi

echo "PM01_EDU_STARTED"
REMOTE
}

remote_start() {
    local pw="$1"
    local remote_script remote_path

    remote_script="$(build_remote_script)"

    # 先本地做语法检查，避免把坏脚本送到 .163
    bash -n -c "$remote_script" ||
        die "生成的远端启动脚本有语法错误"

    remote_path="$(ssh -T "${SSH_OPTS[@]}" "$MOTION_HOST" \
        'mktemp /tmp/pm01_edu_up.XXXXXX')" ||
        die "无法在 .163 创建临时脚本（SSH 是否可用？）"

    if ! ssh -T "${SSH_OPTS[@]}" "$MOTION_HOST" \
            "cat > '$remote_path'" <<< "$remote_script"; then
        ssh -T "${SSH_OPTS[@]}" "$MOTION_HOST" \
            "rm -f '$remote_path'" 2>/dev/null || true
        die "写入 .163 临时脚本失败"
    fi

    # 密码走 stdin，不出现在命令行/ps 里
    if ! printf '%s\n' "$pw" |
         ssh -T "${SSH_OPTS[@]}" "$MOTION_HOST" \
             "bash '$remote_path'"; then
        ssh -T "${SSH_OPTS[@]}" "$MOTION_HOST" \
            "rm -f '$remote_path'" 2>/dev/null || true
        die "远端启动 $MOTION_PROFILE 失败"
    fi
}

acquire_sudo_password() {
    if [ -n "${PM01_163_SUDO_PASSWORD:-}" ]; then
        SUDO_PW="$PM01_163_SUDO_PASSWORD"
    elif [ -t 0 ]; then
        printf '>> .163 (%s) sudo 密码: ' "$MOTION_HOST" >&2
        IFS= read -rsp '' SUDO_PW
        echo >&2
    else
        die "非交互运行请设置 PM01_163_SUDO_PASSWORD，或先手动起 pm01_edu 后用 start_all.sh --no-motion"
    fi

    if [ -z "$SUDO_PW" ]; then
        die "sudo 密码为空"
    fi
}

ensure_motion_sdk() {
    local running

    running="$(ssh -T "${SSH_OPTS[@]}" "$MOTION_HOST" \
        "pgrep -af '[s]rc_executor.*$MOTION_PROFILE' || true" 2>/dev/null || true)"

    if [ -n "$running" ]; then
        log "检测到 .163 已在运行 $MOTION_PROFILE，直接复用（不需要 sudo）："
        printf '%s\n' "$running" | sed 's/^/    /'
        SDK_TOUCHED=0
        return 0
    fi

    acquire_sudo_password
    log "在 .163 停止 pm_v2，启动 $MOTION_PROFILE ..."
    remote_start "$SUDO_PW"
    SDK_TOUCHED=1
}

remote_ready_probe() {
    # READY: src_executor 在跑且 ROS2 桥已初始化
    # INIT / STARTING: 还在启动
    # NOLOG / STALE_LOG: 进程在跑，但日志缺失或不是这个进程写的
    # NOT_RUNNING: src_executor 和 run_robot.sh 都不在了
    remote_bash "
        pid=\$(pgrep -f '[s]rc_executor.*$MOTION_PROFILE' | head -n 1)
        if [ -n \"\$pid\" ]; then
            if [ ! -f '$MOTION_LOG' ]; then
                echo NOLOG
                exit 0
            fi
            now=\$(date +%s)
            lm=\$(stat -c %Y '$MOTION_LOG' 2>/dev/null || echo 0)
            et=\$(ps -o etimes= -p \"\$pid\" 2>/dev/null | tr -d ' ')
            if [ -n \"\$et\" ] && [ \$((now - lm)) -gt \$((et + 5)) ]; then
                echo STALE_LOG
                exit 0
            fi
            if grep -q 'Ros2ControlNode initialized' '$MOTION_LOG' 2>/dev/null; then
                echo READY
            else
                echo INIT
            fi
        elif pgrep -f '[r]un_robot.sh $MOTION_PROFILE' >/dev/null; then
            echo STARTING
        else
            echo NOT_RUNNING
        fi
    " || echo UNKNOWN
}

wait_for_edu_ready() {
    local timeout_s="$1"
    local deadline=$((SECONDS + timeout_s))
    local probe

    while [ "$SECONDS" -lt "$deadline" ]; do
        probe="$(remote_ready_probe || true)"
        case "$probe" in
            READY) return 0 ;;
            NOLOG|STALE_LOG)
                die "检测到 $MOTION_PROFILE 在运行，但 $MOTION_LOG 缺失或不是当前进程写的；无法确认状态，拒绝继续。请先停掉它，让 motion_up.sh 重新启动"
                ;;
            NOT_RUNNING)
                die "$MOTION_PROFILE 进程已退出，没有进入 walk"
                ;;
        esac
        sleep 2
    done
    return 1
}

switch_to_walk() {
    local state="$1"

    case "$state" in
        walk)
            log "当前已在 walk，跳过状态切换"
            ;;
        pd_stand)
            step_to walk 20
            ;;
        passive)
            step_to pd_stand 30
            step_to walk 20
            ;;
        idle)
            step_to passive 10
            step_to pd_stand 30
            step_to walk 20
            ;;
        *)
            warn "当前状态 '$state' 不在 pm01_edu 的 idle→passive→pd_stand→walk 链上。"
            warn "如果你看到 rl_basic，说明 .163 上跑的还是 pm_v2。"
            print_motion_info >&2 || true
            exit 1
            ;;
    esac
}

verify_cmd_vel() {
    log "检查 /cmd_vel 的 ros2_control_node 订阅者 ..."
    if ! wait_cmd_vel_subscriber 20; then
        timeout 5 ros2 topic info /cmd_vel -v --no-daemon >&2 2>/dev/null || true
        die "/cmd_vel 没有 ros2_control_node 订阅者，跟随链路不会生效"
    fi
    log "✓ /cmd_vel 已由 ros2_control_node 订阅"
}

# ------------------------------------------------------------
# 参数
# ------------------------------------------------------------

for arg in "$@"; do
    case "$arg" in
        -y|--yes)     ASSUME_YES=1 ;;
        --check)      CHECK_ONLY=1 ;;
        --dry-run)    DRY_RUN=1 ;;
        -h|--help)    usage; exit 0 ;;
        *)
            printf '未知参数: %s\n' "$arg" >&2
            usage >&2
            exit 2
            ;;
    esac
done

# ------------------------------------------------------------
# 只检查当前状态
# ------------------------------------------------------------

if [ "$CHECK_ONLY" -eq 1 ]; then
    state="$(read_motion_state)"
    if [ "$state" != "walk" ]; then
        warn "当前运控状态: ${state:-未知}（需要 walk）"
        print_motion_info >&2 || true
        exit 1
    fi
    if ! wait_motion_cmd_subscriber 20; then
        warn "/motion/set_motion_state 没有订阅者（pm01_edu 的 ROS2 桥没起来）"
        exit 1
    fi
    verify_cmd_vel
    log "运控就绪: walk + /cmd_vel"
    exit 0
fi

# ------------------------------------------------------------
# 干跑
# ------------------------------------------------------------

if [ "$DRY_RUN" -eq 1 ]; then
    log "DRY-RUN: ssh $MOTION_HOST"
    log "DRY-RUN:   sudo systemctl stop robotics.service"
    log "DRY-RUN:   cd $MOTION_SDK_DIR && ./run_robot.sh $MOTION_PROFILE"
    log "DRY-RUN: 状态链 idle → passive → pd_stand → walk"
    log "DRY-RUN: 校验 /cmd_vel 有 ros2_control_node 订阅者"
    echo "--- 远端脚本 ---"
    build_remote_script | sed 's/^/    /'
    exit 0
fi

# ------------------------------------------------------------
# 安全确认
# ------------------------------------------------------------

if [ "$ASSUME_YES" -ne 1 ]; then
    cat <<'EOF'
>> 即将在 .163 上:
>>   1. 停掉原厂 pm_v2 (robotics.service)
>>   2. 启动 pm01_edu
>>   3. 执行状态切换 idle → passive → pd_stand → walk
>>
>> 请确认:
>>   - 机器人已趴下/有支撑，周围清空，急停可用；
>>   - passive 会松关节，pd_stand 会站起，walk 会进入行走控制；
>>   - 人先离开机器人正前方。
EOF
    read -r -p ">> 输入 yes 继续: " answer
    if [ "$answer" != "yes" ]; then
        echo ">> 已取消。"
        exit 1
    fi
fi

# ------------------------------------------------------------
# 主流程
#
# sudo 密码只在需要真正启动 pm01_edu 时才取（复用在跑的进程不需要）；
# 密码不写入文件、不出现在命令行。
# ------------------------------------------------------------

if ! ssh -T "${SSH_OPTS[@]}" "$MOTION_HOST" true; then
    die "SSH 到 $MOTION_HOST 失败（确认本机到 .163 的免密 key 可用）"
fi

ensure_motion_sdk

log "等待 .163 的 $MOTION_PROFILE ROS2 桥就绪（日志: $MOTION_LOG，最多 120 秒）..."
if ! wait_for_edu_ready 120; then
    die "120 秒内 $MOTION_PROFILE 没有完成初始化"
fi

if ! wait_motion_cmd_subscriber 20; then
    timeout 5 ros2 topic info /motion/set_motion_state -v --no-daemon >&2 2>/dev/null || true
    die "/motion/set_motion_state 没有订阅者，pm01_edu 的 ROS2 桥没起来"
fi
log "✓ /motion/set_motion_state 已由 pm01_edu 订阅"

state="$(read_motion_state)"
if [ -z "$state" ]; then
    print_motion_info >&2 || true
    die "无法确认当前运控状态，拒绝发切换命令"
fi
log "当前运控状态: $state（来源: .163 日志）"

switch_to_walk "$state"
verify_cmd_vel

log "运控就绪: walk + /cmd_vel"
