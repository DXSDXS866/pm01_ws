# PM01 统一工作区 `~/pm01_ws`

把原先分散在 4 个工作区的 PM01 本机代码整合到一个 colcon 工作区：

| 包 | 来源工作区 | 作用 |
|---|---|---|
| `interface_protocol` | `~/engineai_ros2_ws` | 消息/服务定义，含 `MotionGesture.srv` |
| `voice_interaction` | `~/voice_interaction_test_ws` | 唤醒词、ASR、Qwen function call、TTS、动作桥 |
| `pm01_camera_bringup` | `~/ros2_ws` | RealSense D435i（已启用 align_depth） |
| `yolo_person_3d` | `~/ros2_ws` | YOLO26n + TensorRT，输出 `/person/position_robot` |
| `pm01_aruco_detection` | `~/vision_follow_pm01_ws` | 跟随控制器 + `/cmd_vel` 安全桥 |

旧工作区和旧启动脚本**未改动**，可以继续使用；本工作区是独立副本。

另外新增 `motion_up.sh`：通过 SSH 在 `.163` 上停掉原厂 `pm_v2`、启动
`pm01_edu`，并按 `idle → passive → pd_stand → walk` 切到行走态。
`start_all.sh` 默认已包含这一步。

## 构建

```bash
~/pm01_ws/build.sh
```

`build.sh` 会在干净环境（`env -i` + 只 source `/opt/ros/humble`）中构建，
因此 `install/setup.bash` 的 prefix chain 不会把旧工作区烤进去。
构建完成后脚本会自动检查并打印 prefix chain。

增量构建某个包：

```bash
~/pm01_ws/build.sh --packages-select yolo_person_3d
```

## 环境

```bash
source ~/pm01_ws/env.sh
```

`env.sh` 会：

1. 从 `AMENT_PREFIX_PATH` / `CMAKE_PREFIX_PATH` / `COLCON_PREFIX_PATH` /
   `PYTHONPATH` / `LD_LIBRARY_PATH` / `PKG_CONFIG_PATH` 中剔除旧工作区前缀；
2. source `/opt/ros/humble` 和 `/app/applications`（`robot_manager` 等厂商包）；
3. **最后** source `~/pm01_ws/install/setup.bash`，确保 `interface_protocol`
   使用本工作区的版本；
4. 设置 `ROS_DOMAIN_ID=69`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、
   `ROS_LOCALHOST_ONLY=0`、CycloneDDS 网卡 `192.168.0.162`；
5. 取消 `PYTHONNOUSERSITE`，保证 `pycuda` / `numpy 1.26.4` 的 user-site 可用。

> `env.sh` 不设置 `DASHSCOPE_API_KEY`，语音启动脚本会检查它是否已在环境中。

## 运行

### 一键启动（含 .163 运控）

```bash
~/pm01_ws/start_all.sh
```

顺序：

0. SSH 到 `.163`，停原厂 `robotics.service`（`pm_v2`），启动 `pm01_edu`；
   等 `/motion/motion_state` 出现，按 `idle → passive → pd_stand → walk`
   切换，并确认 `/cmd_vel` 有 `ros2_control_node` 订阅者；
1. 相机 + YOLO；
2. 跟随控制（`armed=true`）；
3. 语音交互。

第一次运行需要：

- 本机 `ubuntu` 到 `.163` 的 `user` 免密 SSH（当前已配置）；
- `.163` 上 `user` 的 sudo 密码，脚本会交互提示（不写入磁盘/命令行）。

后台/非交互运行：

```bash
mkdir -p /tmp/pm01_follow_voice
PM01_163_SUDO_PASSWORD='...' \
  nohup ~/pm01_ws/start_all.sh -y > /tmp/pm01_follow_voice/start.log 2>&1 &
```

> 密码放在环境变量里会出现在进程环境中，仅建议在可信的本机使用；
> 更稳妥的做法是给 `.163` 的 `user` 配置受限的 NOPASSWD sudo。

其他入口：

```bash
# 已经在 .163 手动起好 pm01_edu 并切到 walk，只校验，不重复启动
~/pm01_ws/start_all.sh --no-motion

# 只启动/切换 .163 运控，不启动感知/跟随/语音
~/pm01_ws/start_all.sh --motion-only

# 单独运行运控阶段（交互确认 + 密码）
~/pm01_ws/motion_up.sh

# 只检查当前是否 walk + /cmd_vel 订阅者
~/pm01_ws/motion_up.sh --check

# 只打印将执行的步骤，不连 .163
~/pm01_ws/motion_up.sh --dry-run

# 备用入口：以 systemd 单元 pm01-edu 管理 .163 运控
~/pm01_ws/motion_163.sh status
~/pm01_ws/motion_163.sh start          # 停 pm_v2，起 pm01_edu
~/pm01_ws/motion_163.sh switch-walk    # idle → passive → pd_stand → walk
~/pm01_ws/motion_163.sh restore        # 停 pm01_edu，恢复原厂 pm_v2
```

`motion_163.sh` 与 `motion_up.sh` 做同一件事，但使用 `systemd-run` 单元
（`PM01_MOTION_UNIT`，默认 `pm01-edu`）托管远端进程；`motion_up.sh` 使用
`setsid nohup`，并被 `start_all.sh` 直接调用。日常一键启动走
`start_all.sh` / `motion_up.sh` 即可。

日志：`/tmp/pm01_follow_voice/{motion,vision,follow,voice}.log`。

只启动语音（不碰运控/感知）：

```bash
~/pm01_ws/run_voice.sh
```

语音控制跟随：

- “你好小智，跟着我” → `set_follow(true)`
- “你好小智，停止跟随” → `set_follow(false)`，保持站立

## 安全注意

- `start_all.sh` 现在会自动操作 `.163` 运控：
  停 `pm_v2` → 起 `pm01_edu` → `passive`（关节松）→ `pd_stand`（站起）
  → `walk`（行走控制）。启动前必须确认机器人已趴下/有支撑、周围清空、
  急停可用，人先离开正前方。
- 本机 Ctrl+C 只停本机节点；`.163` 的 `pm01_edu` 会继续运行，机器人仍在
  `walk`。脚本退出时会打印切 `pd_stand` / 停 `pm01_edu` / 恢复 `pm_v2`
  的命令。
- `start_all.sh` 使用 `armed=true`，跟随启动后只要画面里有人就会发速度。
  第一次验证建议用 `--no-motion` + 分步启动，或先 `motion_up.sh --check`。
- 两套运控状态名不要混：
  - 原厂 `pm_v2`（`robotics.service`）：`rl_basic`，速度入口
    `/motion/body_vel_cmd`；
  - `pm01_edu`（`run_robot.sh pm01_edu`）：`walk`，速度入口 `/cmd_vel`。
  跟随链路只对 `pm01_edu` 生效。
- 真机 `ROS_DOMAIN_ID=69`；不要在机器人旁边随意启动会发 `/cmd_vel` 的节点。
- 运控侧 `stop_vel_threshold: 0.3`：低于 0.3 的速度指令会被当“停下”。
  跟随 launch 默认 `max_linear_mps:=0.45`、`max_angular_rps:=0.6`，不要随意调低。
- 状态机不能跳级：`idle → passive → pd_stand → walk`；`motion_up.sh`
  已按这个顺序逐级等待确认。
- YOLO 使用 TensorRT `.engine` + pycuda（桌面 torch 无 sm_87），不要换成
  ultralytics 的 TRT 后端。
- `numpy` 必须 < 2，否则 `cv_bridge` 会挂；`PYTHONNOUSERSITE` 会让
  `~/.local` 里的 pycuda/numpy 消失，`env.sh` 已主动 unset。
- `CYCLONEDDS_URI` 里的 `192.168.0.162` 是当前 Jetson 网卡地址；换网段时改
  `env.sh` 这一行。

## 与旧工作区的关系

| 旧路径 | 状态 |
|---|---|
| `~/engineai_ros2_ws` | 保留（`interface_protocol` 已复制） |
| `~/voice_interaction_test_ws` | 保留 |
| `~/ros2_ws` | 保留 |
| `~/vision_follow_pm01_ws` | 保留 |
| `~/start_follow_voice.sh` / `~/start_follow_voice_bg.sh` | 未修改，仍走旧工作区 |

不要在新终端里把旧工作区再 source 到 `~/pm01_ws/env.sh` 之后；那会让
`interface_protocol` 又指向 `/app/applications` 或旧 engineai 版本。

## 已知问题（与整合无关）

- `pm01_aruco_detection` 的 `test/test_aruco_core.py` 有 3 个用例在
  `cv2 5.0.0` 下失败：`IndexError: invalid index to scalar variable`
  （`aruco_core.py:110` 对 `ids` 形状的假设与新版 OpenCV 不一致）。
  旧工作区 `~/vision_follow_pm01_ws` 同样失败，不是本次整合引入的。
  当前 YOLO 跟随链路不经过 ArUco 检测；需要修时把该行改成
  `marker_id = int(np.asarray(marker_id_array).reshape(-1)[0])`。
  其余 12 个用例通过（`test_person_follow_control.py` 5/5、
  `test_pm01_gamepad_bridge.py` 4/4、`test_aruco_core.py` 3/6）。

## 回滚

新工作区完全独立：

```bash
rm -rf ~/pm01_ws
```

旧工作区、`.bashrc`、旧启动脚本都不受影响。

## 故障排查

- **`not found: .../install/local_setup.bash`**：说明旧工作区被 source 进新环境。
  新开一个终端，只 `source ~/pm01_ws/env.sh`；不要先 source 旧工作区。
- **`ModuleNotFoundError: interface_protocol`**：确认 `env.sh` 最后 source 的是
  `~/pm01_ws/install/setup.bash`，并且已运行 `build.sh`。
- **`No module named 'pycuda'` / `cv_bridge` 报 numpy ABI 错误**：不要设置
  `PYTHONNOUSERSITE=1`；`env.sh` 已 unset，若手动设置请取消。
- **YOLO 找不到 engine**：`ls -l ~/pm01_ws/install/yolo_person_3d/share/yolo_person_3d/weights/`，
  或 `~/pm01_ws/build.sh --packages-select yolo_person_3d` 重装 weights。
- **语音播放没声音**：确认 `DASHSCOPE_API_KEY` 已设置；`run_voice.sh` 会打印
  实际 `llm_client` 路径，确认不是旧工作区。
- **`motion_up.sh` SSH 失败**：先手动 `ssh user@192.168.0.163 true`。
  脚本使用免密 key（`BatchMode=yes`），不会交互问 SSH 密码。
- **`motion_up.sh` 提示需要 sudo 密码**：交互终端会提示输入；后台运行用
  `PM01_163_SUDO_PASSWORD`，或先手动起好 `pm01_edu` 再用
  `start_all.sh --no-motion`。
- **120 秒内没有看到 `/motion/motion_state`**：看 `.163` 日志：
  `ssh user@192.168.0.163 'tail -n 60 /tmp/engineai_pm01_edu.log'`。
  `run_robot.sh` 会先检查 motor service、路由（最长 60 秒）和
  `src_executor`；有旧进程时会拒绝启动。
- **状态卡在 `rl_basic`**：说明 `.163` 上还是 `pm_v2`，不是 `pm01_edu`。
  检查 `ps -ef | grep src_executor` 和 `/cmd_vel` 的订阅者。
- **`/cmd_vel` 没有 `ros2_control_node` 订阅者**：`pm01_edu` 没起完整，
  或还跑着 `pm_v2`。跟随链路会完全无效。
- **退出脚本后机器人还在 walk**：这是预期行为，`.163` 的 `pm01_edu`
  不随本机脚本退出。按脚本打印的命令切 `pd_stand` 或停 `pm01_edu`。
- **构建日志**：`~/pm01_ws/log/latest_build/<pkg>/stderr.log`。
