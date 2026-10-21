# 🎙️ voice_interaction — PM01 语音交互 ROS 2 包

PM01-EDU 人形机器人的语音交互节点，部署在机器人计算单元（Jetson Orin NX）。

一条链路打通 **语音 → 意图 → 动作 → 语音回复**：

- **听**：本地麦克风 + 能量 VAD + 阿里 Paraformer 流式 ASR
- **想**：Qwen function call 解析意图（默认 `qwen-flash`，低延迟）
- **做**：`MotionBridge` 把工具调用翻译成 ROS 2 运控指令
- **说**：CosyVoice 流式 TTS，首包即播（`mpg123`），支持唤醒词打断

> [!WARNING]
> 本包会控制真实机器人。真机测试前请确认机器人有支撑、周围清空、急停可用。

## 📦 依赖

- ROS 2 Humble（机器人计算单元自带）
- `interface_protocol` 消息包（本仓库 `src/interface_protocol`，或 Native SDK 自带版本）
- Python：`dashscope`、`pyaudio`、`numpy<2`；可选 `webrtcvad`
- 系统：`mpg123`（流式 TTS）、`ffmpeg`（回退播放）、`alsa-utils`
- 阿里云 DashScope API Key：`DASHSCOPE_API_KEY`

```bash
sudo apt install mpg123 ffmpeg alsa-utils
pip3 install 'numpy<2' dashscope pyaudio sounddevice
```

## 🗂️ 目录结构

```text
voice_interaction/
├── package.xml
├── setup.py / setup.cfg
├── resource/voice_interaction
├── launch/voice_interaction.launch.py
├── config/params.yaml
├── README.md
├── docs/技术文档.md
└── voice_interaction/
    ├── __init__.py
    ├── voice_node.py        # 主节点（IDLE → RECORDING → PROCESSING）
    ├── audio_io.py          # 麦克风采集 + 扬声器播放（支持打断）
    ├── vad.py               # VAD 断句（能量阈值，可选 webrtcvad）
    ├── asr_client.py        # Paraformer 流式 ASR
    ├── llm_client.py        # Qwen function call + 对话历史
    ├── tts_client.py        # CosyVoice 流式 TTS
    ├── media_player.py      # 本地提示音播放
    └── motion_bridge.py     # ROS 2 动作桥
```

## 🚀 在统一工作区中运行（推荐）

本包已包含在 `pm01_ws` 中，不需要单独拷贝：

```bash
cd ~/pm01_ws
./build.sh --packages-select voice_interaction
export DASHSCOPE_API_KEY='...'
./run_voice.sh
```

`run_voice.sh` 会：

1. `source env.sh` 加载工作区环境；
2. 检查 `DASHSCOPE_API_KEY`；
3. 打印实际加载的 `llm_client` 路径，确认不是旧工作区；
4. 启动 `voice_interaction` 节点。

手动启动：

```bash
source ./env.sh
ros2 launch voice_interaction voice_interaction.launch.py
```

## 🧰 独立部署（可选）

如果只想把本包放进一个独立工作区：

```bash
# 1. 拷包
scp -r voice_interaction/ ubuntu@<robot-ip>:~/voice_interaction_ws/src/

# 2. 同时放入 interface_protocol
cp -r ~/engineai_robotics_native_sdk/src/protocol/interface_protocol \
      ~/voice_interaction_ws/src/

# 3. 构建
cd ~/voice_interaction_ws
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=69
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --packages-select voice_interaction
source install/setup.bash
ros2 launch voice_interaction voice_interaction.launch.py
```

## 🎛️ 参数调优

`config/params.yaml`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `energy_threshold` | 500 | VAD 能量阈值。先录环境噪声，设为其 RMS 的 2~3 倍 |
| `silence_ms` | 450 | 句尾静音时长。说话慢可调到 800~1000；句子常被截断也调大 |
| `mic_device_index` | -1 | 麦克风索引，-1=系统默认；`python3 -m sounddevice` 查看 |
| `llm_model` | qwen-flash | 低延迟默认；复杂问答可换 qwen-plus / qwen-max |
| `tts_model` | cosyvoice-v2 | **换 voice 必须同步换 model** |
| `tts_voice` | longshu_v2 | 温暖男声；v2 下可选 `longxiang_v2` / `longxiaochen_v2` 等 |
| `tts_streaming` | true | 首包即播；无 `mpg123` 或想回退旧行为时设 false |

命令行临时覆盖：

```bash
ros2 launch voice_interaction voice_interaction.launch.py silence_ms:=600
```

## 🔌 动作与跟随接口

| 工具 | ROS 2 接口 | 说明 |
|---|---|---|
| `walk` | `/motion/body_vel_cmd`（`BodyVelCmd`） | 需运控处于支持的行走态，100 Hz 持续发 |
| `switch_motion` | `/motion/set_motion_state`（`MotionStateRequest`） | 状态必须在 `available_transition_motions` 内 |
| `wave_hand` | `/motion/joint_override_command` | 需 `lower_body_balance` |
| `gesture` | `/motion/gesture`（服务） | 交警手势 0–8 |
| `emergency_stop` | `/motion/set_motion_state` | 任意状态 → `passive` |
| `set_follow` | `/vision_follow/set_follow` + `/person/set_target` | 锁定单人并跟随 |
| `relock_target` | `/person/set_target` | 重新选择跟随目标 |

## 🎤 测试口令

启动后先说唤醒词 **“你好小智”**，再说：

| 口令 | 预期 |
|---|---|
| “你好小智，跟着我” | 锁定正前方的人并开始跟随 |
| “重新锁定” | 跟随中重新选人 |
| “停止跟随” | 解除锁定并停车站立 |
| “挥挥手” | `wave_hand`（需先站立/平衡态） |
| “做个停止手势” | 交警手势 1 |
| “急停” | `emergency_stop` → `passive` |
| “你好” | 普通闲聊（LLM 回复） |

> 本机主要行走方式是视觉跟随；`walk` 工具保留，但默认提示词优先使用 `set_follow`。

## 🩺 常见问题

| 问题 | 排查 |
|---|---|
| 编译找不到 `interface_protocol` | 按「独立部署」把 msg 包放进工作区，或先 source Native SDK 的 `ros2_env` |
| ASR 无结果 | 检查 `DASHSCOPE_API_KEY`；`arecord test.wav` 确认麦克风有声；调低 `energy_threshold` |
| TTS 无声 | 确认 `mpg123` / `ffmpeg` 已安装；`cosyvoice-v2` 必须配 `*_v2` voice；看日志“TTS 首音” |
| 机器人不动 | 先跑 `./motion_up.sh --check`（`pm01_edu` 的状态从远端日志读）；确认已到 `walk`；再看 `/vision_follow/safety_bridge_status`；检查 `ROS_DOMAIN_ID=69` |
| 误触发太多 | 调高 `energy_threshold`；缩短唤醒窗口；后续可接入专用 KWS 引擎 |
| 打断不生效 | 确认调的是 `TTSClient.stop()`，不是只调 `SpeakerPlayer.stop()` |

## 🛡️ 安全提示

- 真机测试先吊架或空旷场地，速度从低速开始
- `emergency_stop` 随时可用，会立即切 `passive`
- `walk` 没有自动超时，停步必须主动发停止指令或切状态
- 麦克风持续监听，注意隐私（云端 ASR 会上传音频）
- 不要把 `DASHSCOPE_API_KEY` 写入仓库或脚本
