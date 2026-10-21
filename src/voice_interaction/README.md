# voice_interaction — PM01 语音交互 ROS2 包

PM01-EDU 人形机器人的语音交互节点，部署在机器人计算单元（Jetson Orin NX）。
对话+动作联动：阿里通义 ASR/LLM/TTS 全云端，LLM function call 通过 ROS2 控制机器人动作。

## 依赖

- 机器人计算单元上的 ROS2 Humble 环境（机器人自带）
- `interface_protocol` msg 包（编译 Native SDK 时自动生成，路径 `engineai_robotics_native_sdk/src/protocol/interface_protocol`）
- Python 依赖：`dashscope` `pyaudio` `numpy`，可选 `webrtcvad`
- `ffmpeg`（含 ffplay，用于整段 MP3 播放回退路径）：`sudo apt install ffmpeg`
- `mpg123`（推荐，用于流式 TTS 首包即播）：`sudo apt install mpg123`
- 阿里云 DashScope API Key

## 目录结构

```
voice_interaction/
├── package.xml
├── setup.py / setup.cfg
├── resource/voice_interaction
├── launch/voice_interaction.launch.py
├── config/params.yaml
├── README.md
└── voice_interaction/
    ├── __init__.py
    ├── voice_node.py       主节点（状态机：IDLE→RECORDING→PROCESSING）
    ├── audio_io.py         麦克风采集 + 扬声器播放（支持打断）
    ├── vad.py              VAD 断句（能量阈值，可选 webrtcvad）
    ├── asr_client.py       阿里 Paraformer 流式 ASR
    ├── llm_client.py       Qwen function call + 对话历史
    ├── tts_client.py       阿里 CosyVoice 流式 TTS
    └── motion_bridge.py    ROS2 动作桥（walk持续发/switch_motion/wave_hand/emergency_stop）
```

## 部署步骤

### 1. 拷包到机器人计算单元

把整个 `voice_interaction/` 目录拷到机器人的工作区：

```bash
# 在你的 PC 上（假设能 ssh 机器人）
scp -r voice_interaction/ ubuntu@<robot-ip>:~/voice_interaction_ws/src/

# 或者用 U盘 / 共享文件夹
```

### 2. 确保 interface_protocol 可被本包依赖

`interface_protocol` 是 Native SDK 的 ROS2 msg 包。两种方式让本包能找到它：

**方式A（推荐）**：把 `engineai_robotics_native_sdk/src/protocol/interface_protocol` 也拷到同一个工作区一起编译：
```bash
cp -r ~/engineai_robotics_native_sdk/src/protocol/interface_protocol ~/voice_interaction_ws/src/
```

**方式B**：先在 Native SDK 容器里 `./build.sh`，再 `source build/ros2_env/install/setup.bash`，本包编译时就能找到已生成的 msg。

### 3. 装依赖

```bash
ssh ubuntu@<robot-ip>
pip3 install dashscope pyaudio numpy webrtcvad
sudo apt install ffmpeg
# 确认麦克风
arecord -l
```

### 4. 配置 API Key

```bash
echo 'export DASHSCOPE_API_KEY=sk-你的key' >> ~/.bashrc
source ~/.bashrc
```

### 5. 编译运行

```bash
cd ~/voice_interaction_ws
# ROS2 环境
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=69
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

colcon build --packages-select voice_interaction
source install/setup.bash

# 启动
ros2 launch voice_interaction voice_interaction.launch.py
```

### 6. 测试

启动后节点持续监听。对机器人说话试试：
- "你好" → 闲聊回复
- "向前走" → walk(0.3, 0, 0) + 机器人前进（需先 pd_stand→rl_basic）
- "停" → walk(0,0,0)
- "挥手" → wave_hand（需先 lower_body_balance）
- "急停" → emergency_stop → passive

## 参数调优（config/params.yaml）

| 参数 | 说明 | 调优建议 |
|------|------|---------|
| energy_threshold | VAD 能量阈值 | 先录环境噪声，设为其 RMS 的 2~3 倍。误触发就调高，听不到就调低 |
| silence_ms | 句尾静音时长 | 默认 450；说话慢的人调大(800~1000)，想更快响应可再调小 |
| mic_device_index | 麦克风索引 | -1=默认；`python3 -m sounddevice` 查列表 |
| llm_model | LLM 模型 | 默认 qwen-flash（低延迟）；复杂问答可换 qwen-plus/qwen-max |
| tts_voice | 发音人 | 换 tts_model 必须同步换 voice |
| tts_streaming | 流式 TTS 播放 | 默认 true，首包即播；无 mpg123 或想回退旧行为时设 false |

## 动作接口（motion_bridge.py）

| 工具 | ROS2 Topic | msg | 状态前置 |
|------|-----------|-----|---------|
| walk | /motion/body_vel_cmd | BodyVelCmd | rl_basic/rl_amp/rl_terrain（10Hz持续发） |
| switch_motion | /motion/set_motion_state | MotionStateRequest | 在 available_transition_motions 内 |
| wave_hand | /motion/joint_override_command | JointOverrideCommand | lower_body_balance |
| emergency_stop | /motion/set_motion_state | MotionStateRequest | 任意→passive |

## 常见问题

**Q: 编译报找不到 interface_protocol？**
A: 按"部署步骤2"把 interface_protocol 放进工作区，或 source Native SDK 的 ros2_env。

**Q: ASR 无结果？**
A: 检查 DASHSCOPE_API_KEY 环境变量；`arecord test.wav` 确认麦克风有声；调低 energy_threshold。

**Q: TTS 无声？**
A: 确认装了 `mpg123`（流式）和 `ffmpeg`（回退路径）；cosyvoice-v3-flash 的 voice 必须是 longanhuan。

**Q: 机器人不动？**
A: `ros2 topic echo /motion/motion_state` 看当前状态；walk 需先 pd_stand 再 rl_basic；检查 ROS_DOMAIN_ID=69。

**Q: 误触发太多？**
A: 调高 energy_threshold；或加唤醒词（后续可集成 porcupene）。

## 安全提示

- 真机测试先吊架或空旷场地，walk 速度从 0.1m/s 开始
- emergency_stop 随时可用，会立即切 passive
- walk 有 2s 超时保护，但停步建议主动发 walk(0,0,0)
- 麦克风持续监听，注意隐私（云端 ASR 会上传音频）
