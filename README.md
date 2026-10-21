# PM01 机器人统一工作区

面向 PM01 人形机器人的 ROS 2 一体化工作区：把**语音交互、人员检测、视觉跟随、运控启动**整合到同一套可构建、可部署、可一键启动的代码里。

- 语音唤醒 → 语音识别 → 大模型意图理解 → 语音合成
- RealSense 相机 + YOLO26n（TensorRT）人员检测与 3D 定位
- 人员跟随控制器 + 速度安全桥
- 远程启动运动控制主机上的 `pm01_edu`，按 `idle → passive → pd_stand → walk` 切状态
- 语音控制跟随开关、行走、挥手、交警手势（0–8）

> ⚠️ 本仓库会驱动真实机器人。首次运行前请完整阅读「安全须知」，确保机器人有支撑、周围清空、急停可用。

---

## 目录

- [系统架构](#系统架构)
- [仓库结构](#仓库结构)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [配置](#配置)
- [运行](#运行)
- [话题与服务](#话题与服务)
- [语音指令](#语音指令)
- [运控状态机](#运控状态机)
- [安全须知](#安全须知)
- [测试](#测试)
- [故障排查](#故障排查)
- [已知问题](#已知问题)
- [许可证](#许可证)

---

## 系统架构

```text
                 ┌──────────────────────────── 语音链路 ────────────────────────────┐
 麦克风 ──► VAD ──► ASR(dashscope) ──► LLM function call ──► TTS(dashscope) ──► 扬声器
                                             │
                                             ├── set_follow ──► /vision_follow/set_follow
                                             ├── walk / switch_motion / wave_hand
                                             └── gesture ─────► /motion/gesture

                 ┌──────────────────────────── 跟随链路 ────────────────────────────┐
 RealSense D435i ──► YOLO26n(TRT) ──► /person/position_robot
                                             │
                                   person_follow_controller
                                             │
                                     /vision_follow/cmd_vel
                                             │
                                   cmd_vel_safety_bridge  ◄── /motion/motion_state
                                             │
                                          /cmd_vel  ──► pm01_edu（运控主机）
```

| 包 | 作用 |
|---|---|
| `interface_protocol` | PM01 消息/服务定义，含 `MotionGesture.srv`、`BodyVelCmd.msg` 等 |
| `voice_interaction` | 唤醒词、VAD、ASR、Qwen function call、TTS、动作桥 |
| `pm01_camera_bringup` | RealSense D435i 启动（彩色 + 深度 + align_depth） |
| `yolo_person_3d` | YOLO26n + TensorRT + pycuda，输出人员 3D 坐标 |
| `pm01_aruco_detection` | 人员跟随控制器、ArUco 检测、`/cmd_vel` 安全桥、手柄桥 |

---

## 仓库结构

```text
.
├── src/
│   ├── interface_protocol/          # 消息与服务定义
│   ├── voice_interaction/           # 语音交互、LLM 工具调用、动作桥
│   │   ├── config/params.yaml       # VAD / ASR / LLM / TTS 参数
│   │   ├── docs/技术文档.md
│   │   ├── media/                   # 本地提示音（可选，.gitkeep 占位）
│   │   └── voice_interaction/
│   ├── pm01_camera_bringup/         # RealSense launch
│   ├── yolo_person_3d/              # 检测 + 3D 定位，weights/ 含 .engine
│   └── pm01_aruco_detection/        # 跟随控制 + 安全桥 + 单测
├── tools/
│   └── fake_person_pub.py           # 隔离域冒烟测试：模拟人员位置
├── build.sh                         # 干净环境构建
├── env.sh                           # 唯一环境入口
├── run_voice.sh                     # 只启动语音
├── start_all.sh                     # 运控 + 感知 + 跟随 + 语音一键启动
├── motion_up.sh                     # 远程启动/切换运控状态
└── motion_163.sh                    # systemd 方式管理远端运控（备用入口）
```

---

## 环境要求

### 硬件

- PM01 人形机器人；本仓库运行在机器人计算单元（Jetson Orin）上
- Intel RealSense D435i（USB 3.0）
- 麦克风 + 扬声器
- 运控主机与计算单元在同一局域网，可 SSH 互通

### 软件

| 依赖 | 说明 |
|---|---|
| Ubuntu 22.04 + ROS 2 Humble | 基础环境 |
| `realsense2_camera` | RealSense ROS 2 驱动 |
| Python 3.10 | 与 Humble 匹配 |
| NumPy < 2 | **必须**，否则 `cv_bridge` 会因 ABI 不匹配崩溃 |
| TensorRT + pycuda | YOLO `.engine` 推理；桌面版 PyTorch 在 Orin 上没有 `sm_87`，不能走 ultralytics 后端 |
| CycloneDDS | `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| OpenCV | ArUco 检测与图像处理 |
| `dashscope` | 阿里云 ASR / LLM / CosyVoice TTS |
| `pyaudio`、`sounddevice` | 录音与设备枚举 |
| `DASHSCOPE_API_KEY` | 语音链路必需，运行前由环境提供 |

安装示例（按实际环境调整）：

```bash
sudo apt install ros-humble-realsense2-camera \
                 python3-opencv python3-numpy python3-pyaudio
pip3 install 'numpy<2' dashscope sounddevice pycuda
```

---

## 快速开始

```bash
# 1. 克隆到工作区（脚本默认按 ~/pm01_ws 工作；其他路径用 PM01_WS 指定）
git clone <本仓库地址> ~/pm01_ws
cd ~/pm01_ws

# 2. 构建（自动在干净环境中执行，避免旧工作区污染 prefix chain）
./build.sh

# 3. 加载环境
source ./env.sh

# 4. 设置语音服务密钥
export DASHSCOPE_API_KEY='...'

# 5. 可选：只启动语音
./run_voice.sh
```

`build.sh` 的要点：

- 用 `env -i` + 只 source `/opt/ros/humble` 重新执行自身；
- `--symlink-install`，改 Python 代码后无需重新构建（但改了 `setup.py` / 消息定义需重建）；
- 构建结束会打印 `install/setup.bash` 的 prefix chain，确认只包含 `/opt/ros/humble` 与本工作区。

增量构建：

```bash
./build.sh --packages-select yolo_person_3d
```

---

## 配置

### 环境变量

| 变量 | 默认值 | 作用 |
|---|---|---|
| `PM01_WS` | `~/pm01_ws` | 工作区路径，`env.sh` 使用 |
| `ROS_DOMAIN_ID` | `69` | ROS 域；真机固定，勿与其它节点混用 |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS 实现 |
| `CYCLONEDDS_URI` | 绑定 `192.168.0.162` | 计算单元网卡地址，换网段时改 `env.sh` |
| `DASHSCOPE_API_KEY` | 无 | 语音 ASR / LLM / TTS 密钥 |
| `PM01_163_HOST` | `user@192.168.0.163` | 运控主机 SSH 目标 |
| `PM01_163_SDK_DIR` | `/home/user/projects/engineai_robotics` | 运控 SDK 目录 |
| `PM01_163_PROFILE` | `pm01_edu` | 运控配置名 |
| `PM01_163_LOG` | `/tmp/engineai_<profile>.log` | 远端日志路径 |
| `PM01_163_SUDO_PASSWORD` | 无（交互提示） | 非交互运行时的 sudo 密码 |
| `PM01_MOTION_HOST` | `user@192.168.0.163` | `motion_163.sh` 的 SSH 目标 |
| `PM01_MOTION_DIR` | `/home/user/projects/engineai_robotics` | `motion_163.sh` 的 SDK 目录 |
| `PM01_MOTION_UNIT` | `pm01-edu` | `motion_163.sh` 的 systemd 单元名 |

> 密码不要写入仓库或脚本；推荐给运控主机的 `user` 配置受限的 `NOPASSWD` sudo。

### 语音参数

`src/voice_interaction/config/params.yaml`：

```yaml
/**:
  ros__parameters:
    energy_threshold: 500   # VAD 能量阈值（按现场噪声调）
    silence_ms: 800         # 句尾静音判定
    mic_device_index: -1    # -1=系统默认；用 python3 -m sounddevice 查看
    llm_model: qwen-plus    # qwen-turbo / qwen-plus / qwen-max
    tts_model: cosyvoice-v2 # 换 voice 必须同步换 model
    tts_voice: longshu_v2   # 可选 longxiang_v2 / longxiaochen_v2 等
```

### 相机与检测参数

- 相机：`src/pm01_camera_bringup/launch/camera.launch.py`（已启用 `align_depth`）
- 检测：`src/yolo_person_3d/launch/person_3d.launch.py`
  - `model_path`：默认指向安装后的 `weights/yolo26n.engine`
  - `device`：默认 `cpu`；GPU 推理走 TensorRT + pycuda
  - `camera_mount_pitch_deg`：默认 `16.1`，按实际安装角度调整
- 跟随：`src/pm01_aruco_detection/launch/person_follow.launch.py`
  - 默认 `desired_distance_m=0.8`、`max_linear_mps=0.45`、`max_angular_rps=0.6`
  - `armed` 默认 `false`，一键启动脚本会显式传 `armed:=true`

---

## 运行

### 一键启动（运控 + 感知 + 跟随 + 语音）

```bash
./start_all.sh
```

启动顺序：

0. SSH 到运控主机：停原厂 `robotics.service`（`pm_v2`）→ 启动 `pm01_edu` → 等 `/motion/motion_state` → 按 `idle → passive → pd_stand → walk` 切换 → 校验 `/cmd_vel` 有 `ros2_control_node` 订阅者
1. 相机 + YOLO
2. 跟随控制（`armed=true`）
3. 语音交互

首次运行需要：计算单元到运控主机的免密 SSH；运控主机上的 sudo 密码（交互输入，不落盘）。

选项：

```bash
./start_all.sh -y             # 跳过交互确认（后台/脚本）
./start_all.sh --no-motion    # 不启动运控，只校验现有 walk
./start_all.sh --motion-only  # 只启动运控并切到 walk
```

后台运行：

```bash
mkdir -p /tmp/pm01_follow_voice
PM01_163_SUDO_PASSWORD='...' \
  nohup ./start_all.sh -y > /tmp/pm01_follow_voice/start.log 2>&1 &
```

日志：`/tmp/pm01_follow_voice/{motion,vision,follow,voice}.log`。

### 只启动语音

```bash
./run_voice.sh
```

脚本会先打印实际 `llm_client` 的 import 路径，用于确认加载的是本工作区代码。

### 分步启动

```bash
source ./env.sh

# 相机 + YOLO
ros2 launch yolo_person_3d bringup.launch.py

# 跟随（确认机器人处于 walk、周围无人后再 arm）
ros2 launch pm01_aruco_detection person_follow.launch.py armed:=true

# 语音
./run_voice.sh
```

只启动 ArUco 检测：

```bash
ros2 launch pm01_aruco_detection aruco_detection.launch.py
```

### 运控管理

```bash
# 交互式启动/切换运控
./motion_up.sh

# 只检查是否处于 walk + /cmd_vel 订阅者
./motion_up.sh --check

# 只打印步骤，不连运控主机
./motion_up.sh --dry-run

# systemd 方式（备用入口）
./motion_163.sh status
./motion_163.sh start          # 停 pm_v2，起 pm01_edu
./motion_163.sh switch-walk    # idle → passive → pd_stand → walk
./motion_163.sh restore        # 停 pm01_edu，恢复 pm_v2
```

### 隔离域冒烟测试（不驱动真机）

在独立 ROS 域、只启动跟随控制器并喂入模拟人员位置：

```bash
source ./env.sh
# env.sh 会设置真机域 69，这里必须覆盖到隔离域
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI

ros2 run pm01_aruco_detection person_follow_controller &
python3 tools/fake_person_pub.py 2.0 0.0
ros2 topic echo --once /vision_follow/cmd_vel
```

---

## 话题与服务

### 关键话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 相机 → 检测 | 彩色图 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 相机 → 检测 | 对齐深度 |
| `/person/detected` | `std_msgs/Bool` | 检测 → 跟随 | 是否检测到人 |
| `/person/position_camera` | `geometry_msgs/PointStamped` | 检测 | 相机坐标系位置 |
| `/person/position_robot` | `geometry_msgs/PointStamped` | 检测 → 跟随 | 机器人坐标系位置 |
| `/vision_follow/set_follow` | `std_msgs/Bool` | 语音 → 跟随 | 跟随开关 |
| `/vision_follow/status` | `std_msgs/String` | 跟随 | 状态 |
| `/vision_follow/cmd_vel` | `geometry_msgs/Twist` | 跟随 → 安全桥 | 期望速度 |
| `/vision_follow/safety_bridge_status` | `std_msgs/String` | 安全桥 | 放行/拦截原因 |
| `/cmd_vel` | `geometry_msgs/Twist` | 安全桥 → 运控 | 最终速度 |
| `/motion/motion_state` | `interface_protocol/MotionState` | 运控 | 当前状态 |
| `/motion/set_motion_state` | `interface_protocol/MotionStateRequest` | → 运控 | 切换状态 |
| `/motion/gesture` | `interface_protocol/MotionGesture`（服务） | 语音 → 运控 | 交警手势 0–8 |

### 主要服务

| 服务 | 说明 |
|---|---|
| `/motion/gesture` | 执行交警手势，`gesture_id` 0–8 |
| `/motion/enable_motor` | 电机使能（`interface_protocol/EnableMotor`，运控侧提供） |

---

## 语音指令

唤醒词：**“你好小智”**（可在语音包中修改）。

| 说法 | 动作 |
|---|---|
| 跟着我 | 打开跟随 |
| 停止跟随 / 别跟着我 / 停下 | 关闭跟随并保持站立（安全指令，不经过 LLM） |
| 向前走 / 后退 | `walk` |
| 站起来 / 趴下 | `switch_motion` |
| 挥挥手 | `wave_hand` |
| 做直行/左转弯/右转弯/停止的交警手势 | `gesture`（0=归位 1=停止 2=直行 3=左转弯 4=左转弯待转 5=右转弯 6=变道 7=减速慢行 8=靠边停车） |

LLM 工具共 5 个：`walk`、`switch_motion`、`wave_hand`、`gesture`、`set_follow`（`emergency_stop` 会映射到 `set_follow(false)`）。

---

## 运控状态机

```text
idle ──► passive ──► pd_stand ──► walk
                                    │
                          /cmd_vel 才生效
```

- 状态**不能跳级**；`motion_up.sh` 会逐级等待确认。
- 两套配置不要混用：

| 配置 | 服务 | 状态名 | 速度入口 |
|---|---|---|---|
| 原厂 `pm_v2` | `robotics.service` | `rl_basic` | `/motion/body_vel_cmd` |
| 本仓库使用 `pm01_edu` | `run_robot.sh pm01_edu` | `walk` | `/cmd_vel` |

- 跟随链路只对 `pm01_edu` 生效。若状态卡在 `rl_basic`，说明运控主机上跑的还是 `pm_v2`。
- 运控侧 `stop_vel_threshold: 0.3`：低于 0.3 的速度指令会被当作“停下”，不要随意调低跟随速度。

---

## 安全须知

- `start_all.sh` 会自动操作真机运控：
  `passive` 会松关节，`pd_stand` 会站起，`walk` 会进入行走控制。
- 启动前必须确认：机器人已趴下或有支撑、周围清空、急停可用、人离开机器人正前方。
- `armed=true` 时，画面里只要有人就会发速度。第一次验证建议用
  `--no-motion` + 分步启动，或先运行 `motion_up.sh --check`。
- Ctrl+C 只停本机节点；运控主机的 `pm01_edu` 会继续运行，机器人仍在 `walk`。
  脚本退出时会打印切回 `pd_stand` / 停止 `pm01_edu` / 恢复 `pm_v2` 的命令。
- 真机 `ROS_DOMAIN_ID=69`；不要在机器人旁随意启动会发布 `/cmd_vel` 的节点。
- `numpy` 必须 < 2；不要设置 `PYTHONNOUSERSITE=1`，否则 user-site 里的
  pycuda / numpy 会不可见。
- YOLO 使用 TensorRT `.engine` + pycuda，不要切换成 ultralytics 的 TRT 后端。

---

## 测试

```bash
source ./env.sh
colcon test --base-paths src \
  --packages-select pm01_aruco_detection \
  --event-handlers console_direct+
colcon test-result --verbose
```

既有结果：`test_person_follow_control.py` 5/5、`test_pm01_gamepad_bridge.py` 4/4、
`test_aruco_core.py` 3/6（3 个失败与 OpenCV 版本有关，见「已知问题」）。

---

## 故障排查

| 现象 | 排查 |
|---|---|
| `not found: .../install/local_setup.bash` | 旧工作区被 source 进来了。新开终端，只 `source ./env.sh` |
| `ModuleNotFoundError: interface_protocol` | 确认 `install/setup.bash` 存在并最后 source；先运行 `./build.sh` |
| `No module named 'pycuda'` / numpy ABI 错误 | 不要设 `PYTHONNOUSERSITE=1`；确认 `numpy<2` |
| YOLO 找不到 engine | `ls install/yolo_person_3d/share/yolo_person_3d/weights/`，或单独重建该包 |
| 语音没声音 | 确认 `DASHSCOPE_API_KEY`；看 `run_voice.sh` 打印的 import 路径 |
| `motion_up.sh` SSH 失败 | 先手动 `ssh user@<运控主机> true`；脚本使用免密 key（`BatchMode=yes`） |
| 提示需要 sudo 密码 | 交互终端会提示；后台用 `PM01_163_SUDO_PASSWORD`，或先手动起 `pm01_edu` 再用 `--no-motion` |
| 120 秒内没有 `/motion/motion_state` | `ssh <运控主机> 'tail -n 60 /tmp/engineai_pm01_edu.log'` |
| 状态卡在 `rl_basic` | 运控主机上还是 `pm_v2`；检查 `ps -ef | grep src_executor` |
| `/cmd_vel` 没有 `ros2_control_node` 订阅者 | `pm01_edu` 未完整启动，或 `pm_v2` 仍在跑；跟随链路无效 |
| 退出脚本后机器人还在 walk | 预期行为；按脚本提示切 `pd_stand` 或停止 `pm01_edu` |
| 构建失败 | 看 `log/latest_build/<pkg>/stderr.log` |

---

## 已知问题

- `pm01_aruco_detection/test/test_aruco_core.py` 在 OpenCV 5.x 下有 3 个用例失败：
  `IndexError: invalid index to scalar variable`（`aruco_core.py` 对 `ids` 形状的假设与新版 OpenCV 不一致）。
  当前 YOLO 跟随链路不经过 ArUco 检测；修复方式是把该行改为
  `marker_id = int(np.asarray(marker_id_array).reshape(-1)[0])`。
- `yolo_person_3d/weights/` 同时保留 `.onnx` 与 `.pt` 便于重新导出；
  运行时只需要 `.engine`。

---

## 许可证

本仓库包含的包分别声明了 MIT / Apache-2.0。厂商 SDK、模型权重与第三方依赖遵循其各自许可证；
使用前请确认你有权在目标设备上部署这些组件。

---

## 相关文档

- 语音实现细节：`src/voice_interaction/docs/技术文档.md`
- 语音包 README：`src/voice_interaction/README.md`
