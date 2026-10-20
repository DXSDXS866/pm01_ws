#!/usr/bin/env bash
# ============================================================
# PM01 统一工作区环境入口
#
# 用法:
#   source ~/pm01_ws/env.sh
#
# 不要用 `set -u` source 本文件（ROS setup.bash 会引用未定义变量）。
# 可重复 source；每次都会先清掉旧工作区前缀，再重新加载本工作区。
# ============================================================

PM01_WS="${PM01_WS:-$HOME/pm01_ws}"
export PM01_WS

# ------------------------------------------------------------
# 1. 从环境变量里剔除旧工作区 / 本工作区的旧前缀
# ------------------------------------------------------------
for _var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
            PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH; do
    _val="${!_var:-}"
    if [ -z "$_val" ]; then
        continue
    fi

    _new=""
    _old_ifs="$IFS"
    IFS=':'
    for _p in $_val; do
        case "$_p" in
            "$PM01_WS"/*|/home/ubuntu/pm01_ws/*|\
            /home/ubuntu/engineai_ros2_ws*|\
            /home/ubuntu/voice_interaction_test_ws*|\
            /home/ubuntu/voice_interaction_ws*|\
            /home/ubuntu/vision_follow_pm01_ws*|\
            /home/ubuntu/ros2_ws*|\
            /home/ubuntu/engineai_ros2_workspace_auto_walk*|\
            /home/ubuntu/vision_validation_20260825*)
                ;;
            *)
                _new="${_new:+$_new:}$_p"
                ;;
        esac
    done
    IFS="$_old_ifs"

    export "$_var=$_new"
done
unset _var _val _new _old_ifs _p

# ------------------------------------------------------------
# 2. 基础 ROS + 厂商应用
# ------------------------------------------------------------
source /opt/ros/humble/setup.bash

# /app/applications 提供 robot_manager 等厂商包；其中也有一个旧版
# interface_protocol，下一步 source 本工作区后会被覆盖。
if [ -f /app/applications/install/setup.bash ]; then
    source /app/applications/install/setup.bash
fi

# ------------------------------------------------------------
# 3. 本工作区必须最后 source（interface_protocol 以本工作区为准）
# ------------------------------------------------------------
if [ ! -f "$PM01_WS/install/setup.bash" ]; then
    echo ">> 未找到 $PM01_WS/install/setup.bash" >&2
    echo ">> 请先运行: $PM01_WS/build.sh" >&2
    return 1 2>/dev/null || exit 1
fi

source "$PM01_WS/install/setup.bash"

# ------------------------------------------------------------
# 4. 与现有 ~/.bashrc 一致的运行参数
# ------------------------------------------------------------
export ROS_DOMAIN_ID=69
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.0.162"/></Interfaces></General></Domain></CycloneDDS>'

# 让 pycuda / numpy 1.26.4 的 user-site 保持可用。
unset PYTHONNOUSERSITE

echo ">> PM01 工作区: $PM01_WS"
echo ">> ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
