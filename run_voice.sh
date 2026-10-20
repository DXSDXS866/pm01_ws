#!/usr/bin/env bash
# ============================================================
# 只启动语音交互（使用 ~/pm01_ws）
#
# 用法:
#   ~/pm01_ws/run_voice.sh
#   ~/pm01_ws/run_voice.sh --show-args
# ============================================================
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"

if [ -z "${DASHSCOPE_API_KEY:-}" ]; then
    echo ">> 错误: 未设置 DASHSCOPE_API_KEY" >&2
    echo ">> 在已加载 ~/.bashrc 的交互终端运行，或在当前 shell export 后再试。" >&2
    exit 1
fi

echo ">> 使用工作区: $PM01_WS"
echo ">> llm_client: $(python3 -c 'import voice_interaction.llm_client as m; print(m.__file__)')"

exec ros2 launch voice_interaction voice_interaction.launch.py "$@"
