#!/usr/bin/env bash
# ============================================================
# PM01 统一工作区构建脚本
#
# 关键点：先在“干净环境”里重新执行自己，再 source /opt/ros/humble。
# 这样生成的 install/setup.bash 只会链 /opt/ros/humble，不会把
# 旧的 ros2_ws / vision_follow / engineai / voice_interaction_test_ws
# 写进 prefix chain。
#
# 用法:
#   ~/pm01_ws/build.sh
#   ~/pm01_ws/build.sh --packages-select yolo_person_3d
# ============================================================

# 不能用 set -u：ROS 的 setup.bash 会引用未定义变量。
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "${PM01_CLEAN_ENV:-0}" != "1" ]; then
    exec /usr/bin/env -i \
        HOME="$HOME" \
        USER="$(id -un)" \
        LOGNAME="$(id -un)" \
        PATH=/usr/local/bin:/usr/bin:/bin \
        LANG="${LANG:-C.UTF-8}" \
        TERM="${TERM:-dumb}" \
        PM01_CLEAN_ENV=1 \
        /bin/bash --noprofile --norc "$WS_DIR/build.sh" "$@"
fi

# 二次保险：清掉可能由父进程带进来的 ROS/colcon 环境。
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
      PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
unset ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY CYCLONEDDS_URI

source /opt/ros/humble/setup.bash

echo ">> 干净环境构建: $WS_DIR"
echo ">> ROS: ${ROS_DISTRO:-unknown}  colcon: $(command -v colcon)"

colcon build \
    --base-paths "$WS_DIR/src" \
    --symlink-install \
    --executor sequential \
    --event-handlers console_direct+ \
    "$@"

echo
echo ">> 构建完成。install/setup.bash 的 prefix chain:"
grep -n 'COLCON_CURRENT_PREFIX=' "$WS_DIR/install/setup.bash" || true

# 防止旧工作区被烤进 prefix chain。
if grep -qE 'voice_interaction_test_ws|voice_interaction_ws|vision_follow_pm01_ws|engineai_ros2_ws|/home/ubuntu/ros2_ws' \
        "$WS_DIR/install/setup.bash"; then
    echo ">> 错误: install/setup.bash 仍包含旧工作区路径。" >&2
    exit 1
fi

echo ">> prefix chain 检查通过（无旧工作区）。"
