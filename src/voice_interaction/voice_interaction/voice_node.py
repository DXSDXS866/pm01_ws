import enum
import json
import os
import re
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .audio_io import MicrophoneRecorder
from .vad import SimpleVAD
from .asr_client import ASRClient
from .llm_client import LLMClient
from .tts_client import TTSClient
from .motion_bridge import GESTURES, MotionBridge


# ============================================================
# 唤醒词
#
# ASR 把“小智”识别成各种同音字（小志/小橙/小澄/…），
# 列举清单永远会漏（“小澄”就不在表里），所以用正则匹配
# “你好小” + 任意一个字，任意同音字/错字都能触发。
# ============================================================

_WAKE_RE = re.compile(r'你好小.')


# 单独说“你好小智”之后
# 等待下一句话的时间
WAKE_TIMEOUT = 8.0


# ============================================================
# 固定 MP3 指令
#
# 命中这些命令：
#
# 直接播放 MP3
# 不进入 LLM
#
# ============================================================

# 跟随模式中被拦截的动作工具。
# 跟随中 LLM 再调这些工具会打乱运控状态机
# （walk 状态不在语音侧状态表里，会被当链外状态
#  请求切 passive 瘫软），一律拦截并提示。
# emergency_stop 特殊处理：转成"停止跟随"（保持站立）。
_FOLLOW_GUARDED_TOOLS = {
    'walk',
    'switch_motion',
    'wave_hand',
    'gesture',
    'emergency_stop',
}


# ============================================================
# 直连指令（不进 LLM）
#
# "停止跟随" 是安全关键指令，走 LLM 工具调用既慢又不可靠
# （2026-08-31 真机实测 LLM 链路没把 set_follow 发出来，
#  跟随桥全程没收到），所以也采用本地关键词直连：
# 关键词命中 -> 直接执行。
#
# 注意匹配顺序：先手势（"停止手势"含"停止"），
# 再停跟随（"别跟着我"含"跟着我"），最后开跟随。
# ============================================================

# 手势名称 -> 手势ID（0=归位 1=停止 2=直行 3=左转弯
# 4=左转弯待转 5=右转弯 6=变道 7=减速慢行 8=靠边停车）
# 长词在前（左转弯待转 > 左转弯 > 左转, 减速慢行 > 减速），
# 文本含"手势"或"交警"时按名称匹配，
# "做个直行的交警手势"这类语序也能命中
GESTURE_NAME_TO_ID = (
    ('归位', 0),
    ('左转弯待转', 4),
    ('左转弯', 3),
    ('左转', 3),
    ('右转弯', 5),
    ('右转', 5),
    ('直行', 2),
    ('停止', 1),
    ('变道', 6),
    ('减速慢行', 7),
    ('减速', 7),
    ('靠边停车', 8),
)

# 停止跟随（优先于开跟随匹配）
FOLLOW_OFF_COMMANDS = (
    '停止跟随',
    '停止跟着',
    '不要跟着',
    '别跟着',
    '别跟了',
    '停止跟踪',
    '站住',
    '停下',
    '别动',
    '停止',
)

# 开始跟随（会同时锁定正前方的人）
FOLLOW_ON_COMMANDS = (
    '跟着我',
    '开始跟随',
    '跟上我',
    '跟上来',
    '继续跟随',
)

# 重新锁定（必须在"开始跟随"之前匹配："重新跟着我"含"跟着我"）
RELOCK_COMMANDS = (
    '重新锁定',
    '重新锁我',
    '重新找我',
    '重新找到我',
    '重新跟着我',
)


class _State(enum.Enum):

    IDLE = 0
    RECORDING = 1


class VoiceInteractionNode(Node):

    def __init__(self):

        super().__init__(
            'voice_interaction'
        )

        # ====================================================
        # ROS2 参数
        # ====================================================

        self.declare_parameter(
            'energy_threshold',
            500
        )

        # 句尾静音：800ms 太保守，语音应答会明显慢半拍；
        # 450ms 在展厅环境实测可用，抢话风险可用参数调回。
        self.declare_parameter(
            'silence_ms',
            450
        )

        self.declare_parameter(
            'mic_device_index',
            -1
        )

        # qwen-flash：实测首 token/function call 比 qwen-plus 快 0.3~1.1s，
        # 手势/讲解工具调用已验证正常；复杂问答可改回 qwen-plus。
        self.declare_parameter(
            'llm_model',
            'qwen-flash'
        )

        self.declare_parameter(
            'tts_model',
            'cosyvoice-v2'
        )

        self.declare_parameter(
            'tts_voice',
            'longshu_v2'
        )

        # 流式 TTS：首包即播，显著降低首音延迟；
        # 无 mpg123 时 TTSClient 会自动退回整段播放。
        self.declare_parameter(
            'tts_streaming',
            True
        )

        # ====================================================
        # 麦克风
        # ====================================================

        mic_index = self.get_parameter(
            'mic_device_index'
        ).value

        self.mic = MicrophoneRecorder(
            device_index=(
                None
                if mic_index < 0
                else mic_index
            )
        )

        # ====================================================
        # VAD
        # ====================================================

        self.vad = SimpleVAD(

            energy_threshold=
            self.get_parameter(
                'energy_threshold'
            ).value,

            silence_duration_ms=
            self.get_parameter(
                'silence_ms'
            ).value,
        )

        # ====================================================
        # ASR
        # ====================================================

        self.asr = ASRClient()

        # ====================================================
        # LLM
        # ====================================================

        self.llm = LLMClient(

            model=
            self.get_parameter(
                'llm_model'
            ).value
        )

        # ====================================================
        # TTS
        # ====================================================

        self.tts = TTSClient(

            self,

            model=
            self.get_parameter(
                'tts_model'
            ).value,

            voice=
            self.get_parameter(
                'tts_voice'
            ).value,

            streaming=
            self.get_parameter(
                'tts_streaming'
            ).value,
        )

        # ====================================================
        # PM01 动作
        # ====================================================

        self.bridge = MotionBridge(
            self
        )

        # ====================================================
        # 跟随模式状态（跟随侧安全桥广播）
        #
        # /vision_follow/safety_bridge_status 是 JSON 字符串，
        # state == "FORWARDING" 表示正在把跟随指令转发给机器人。
        # 跟随期间语音动作指令（walk/switch_motion/挥手/手势）
        # 会被守卫拦截，防止误触发把机器人切到 passive 瘫软。
        # ====================================================

        self._follow_active = False

        # 语音侧请求的跟随状态（armed_source="voice"）。
        # 目标暂时丢失时 _follow_active 会变成 False，
        # 但守卫仍应拦截动作指令。
        self._follow_requested = False

        self.create_subscription(
            String,
            '/vision_follow/safety_bridge_status',
            self._on_follow_status,
            10,
        )

        # ====================================================
        # 单人锁定状态（感知侧 /person/status）
        #
        # lock_state: OFF / PENDING / LOCKED / LOST / LEGACY
        # 说"跟着我"后最多等 1.5 s 再播报锁定结果。
        # ====================================================

        self._person_status = {}
        self._person_status_time = 0.0
        self._person_status_cond = threading.Condition()
        self._follow_cmd_generation = 0
        self._follow_cmd_lock = threading.Lock()

        self.create_subscription(
            String,
            '/person/status',
            self._on_person_status,
            10,
        )

        # ====================================================
        # 麦克风状态
        # ====================================================

        self._state = _State.IDLE

        self._state_lock = (
            threading.Lock()
        )

        # ====================================================
        # 唤醒状态
        # ====================================================

        self._awake_until = 0.0

        # ====================================================
        # 对话编号
        #
        # 每重新唤醒一次
        # 上一轮 LLM 回答自动作废
        # ====================================================

        self._turn_id = 0

        self._turn_lock = (
            threading.Lock()
        )

        # ====================================================
        # 内置 MP3 状态
        # ====================================================

        self._media_active = False

        self._media_lock = (
            threading.Lock()
        )

        # ====================================================
        # 启动日志
        # ====================================================

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            '语音交互节点启动成功'
        )

        self.get_logger().info(
            '请先说：你好小智'
        )

        self.get_logger().info(
            '本地讲解音频可按需放入 media 目录'
        )

        self.get_logger().info(
            f'内置音频目录: '
            f'{self._get_media_dir()}'
        )

        self.get_logger().info(
            '======================================'
        )

        # ====================================================
        # 启动麦克风线程
        # ====================================================

        threading.Thread(

            target=self._mic_loop,

            daemon=True,

        ).start()

    # ========================================================
    # 麦克风主循环
    # ========================================================

    def _mic_loop(self):

        self.mic.start()

        while rclpy.ok():

            try:

                chunk = (
                    self.mic.read_chunk()
                )

            except Exception as e:

                self.get_logger().error(
                    f'麦克风读取失败: {e}'
                )

                break

            # =================================================
            # VAD
            # =================================================

            vad_state = (
                self.vad.process(
                    chunk
                )
            )

            with self._state_lock:

                cur = self._state

            # =================================================
            # IDLE
            # =================================================

            if cur == _State.IDLE:

                if vad_state == 'speaking':

                    try:

                        self.asr.start()

                        self.asr.send_audio(
                            chunk
                        )

                        self._set_state(
                            _State.RECORDING
                        )

                        self.get_logger().info(
                            '▶ 检测到说话，开始识别'
                        )

                    except Exception as e:

                        self.get_logger().error(
                            f'ASR启动失败: {e}'
                        )

                        self.vad.reset()

                        self._set_state(
                            _State.IDLE
                        )

            # =================================================
            # RECORDING
            # =================================================

            elif cur == _State.RECORDING:

                try:

                    self.asr.send_audio(
                        chunk
                    )

                except Exception as e:

                    self.get_logger().error(
                        f'ASR发送失败: {e}'
                    )

                # =================================================
                # 机器人正在说话或者播放 MP3
                #
                # 实时看 ASR partial
                #
                # 一旦识别：
                #
                # 你好小智
                #
                # 马上打断
                # =================================================

                if self._is_output_active():

                    partial = (
                        self._normalize_text(
                            self.asr.get_partial()
                        )
                    )

                    wake_word = (
                        self._find_wake_word(
                            partial
                        )
                    )

                    if wake_word is not None:

                        self.get_logger().info(
                            f'⚡ 打断唤醒词命中: '
                            f'{wake_word}'
                        )

                        try:

                            text = (
                                self.asr.stop()
                                .strip()
                            )

                        except Exception:

                            text = partial

                        self.vad.reset()

                        self._set_state(
                            _State.IDLE
                        )

                        if not text:

                            text = partial

                        self._process_recognized_text(
                            text
                        )

                        continue

                # =================================================
                # 正常一句话结束
                # =================================================

                if (
                    vad_state
                    == 'end_of_utterance'
                ):

                    try:

                        text = (
                            self.asr.stop()
                            .strip()
                        )

                    except Exception as e:

                        self.get_logger().error(
                            f'ASR停止失败: {e}'
                        )

                        text = ''

                    self.vad.reset()

                    self._set_state(
                        _State.IDLE
                    )

                    self.get_logger().info(
                        f'🎤 识别: {text}'
                    )

                    if text:

                        self._process_recognized_text(
                            text
                        )

    # ========================================================
    # 处理 ASR 识别文字
    # ========================================================

    def _process_recognized_text(
        self,
        text: str,
    ):

        text = self._normalize_text(
            text
        )

        if not text:

            return

        # ====================================================
        # 查找唤醒词
        # ====================================================

        wake_word = (
            self._find_wake_word(
                text
            )
        )

        # ====================================================
        # 情况1
        #
        # 当前这句话包含：
        #
        # 你好小智
        # 你好小志
        # 你好小乘
        # ...
        # ====================================================

        if wake_word is not None:

            self.get_logger().info(
                f'✅ 唤醒词命中: '
                f'{wake_word}'
            )

            # ------------------------------------------------
            # 上一轮对话作废
            # ------------------------------------------------

            self._new_turn()

            # ------------------------------------------------
            # 停止当前 TTS / MP3
            # ------------------------------------------------

            self._stop_current_output()

            # ------------------------------------------------
            # 开启唤醒窗口
            # ------------------------------------------------

            self._awake_until = (
                time.time()
                + WAKE_TIMEOUT
            )

            # =================================================
            # 兼容：
            #
            # 介绍你好小志，跟着我
            #
            # 只取“你好小志”后面的内容：
            #
            # 跟着我
            # =================================================

            command = text.split(
                wake_word,
                1
            )[1]

            command = (
                self._normalize_text(
                    command
                )
            )

            # =================================================
            # 只说：
            #
            # 你好小智
            #
            # 等待下一句话
            # =================================================

            if not command:

                self.get_logger().info(
                    f'👂 已唤醒，请在 '
                    f'{WAKE_TIMEOUT:.0f} '
                    f'秒内说下一句'
                )

                return

            # =================================================
            # 一句话直接：
            #
            # 你好小智跟着我
            # =================================================

            self._execute_command(
                command
            )

            return

        # ====================================================
        # 情况2
        #
        # 前面已经单独说过你好小智
        # ====================================================

        if (
            time.time()
            < self._awake_until
        ):

            self._execute_command(
                text
            )

            return

        # ====================================================
        # 情况3
        #
        # 没有唤醒
        # 不进入 LLM
        # ====================================================

        self.get_logger().info(
            f'💤 未唤醒，忽略: {text}'
        )

    # ========================================================
    # 执行唤醒后的命令
    # ========================================================

    def _execute_command(
        self,
        text: str,
    ):

        text = self._normalize_text(
            text
        )

        # ----------------------------------------------------
        # 一次唤醒只执行一条命令
        # ----------------------------------------------------

        self._awake_until = 0.0

        if not text:

            return

        # ----------------------------------------------------
        # 创建新的对话轮次
        # ----------------------------------------------------

        turn_id = self._new_turn()

        self.get_logger().info(
            f'👉 执行指令: {text}'
        )

        # ====================================================
        # 第零优先级
        #
        # 直连指令（不进 LLM，零延迟）
        #
        # 跟随开关是安全关键指令，LLM 链路慢且不可靠
        # （2026-08-31 实测没发出 set_follow），直接执行。
        # ====================================================

        direct = (
            self._match_direct_command(
                text
            )
        )

        if direct is not None:

            self.get_logger().info(
                f'⚡ 直连指令命中: {direct}'
            )

            # set_follow 发布很快，但后续 TTS 是阻塞式
            # （网络合成+播放），绝不能堵麦克风线程
            threading.Thread(

                target=
                self._execute_direct_command,

                args=(
                    direct,
                ),

                daemon=True,

            ).start()

            return

        # ====================================================
        # 固定 MP3 / 普通问题 统一交给 LLM 处理
        # ====================================================

        threading.Thread(

            target=
            self._handle_utterance,

            args=(
                text,
                turn_id,
            ),

            daemon=True,

        ).start()

    # ========================================================
    # 直连指令：匹配 + 执行
    #
    # 返回 (kind, value) 或 None：
    #   ('gesture', id) / ('follow_off', None) / ('follow_on', None)
    # ========================================================

    def _match_direct_command(
        self,
        text: str,
    ):

        # ------------------------------------------------
        # 手势最先匹配："停止手势"含"停止"，
        # 必须抢在停跟随之前
        # ------------------------------------------------

        if '手势' in text or '交警' in text:

            for name, gesture_id in GESTURE_NAME_TO_ID:

                if name in text:

                    return ('gesture', gesture_id)

        # ------------------------------------------------
        # 重新锁定先于停/开跟随："重新跟着我"含"跟着我"
        # ------------------------------------------------

        for keyword in RELOCK_COMMANDS:

            if keyword in text:

                return ('relock', None)

        # ------------------------------------------------
        # 停跟随先于开跟随："别跟着我"含"跟着我"
        # ------------------------------------------------

        for keyword in FOLLOW_OFF_COMMANDS:

            if keyword in text:

                return ('follow_off', None)

        for keyword in FOLLOW_ON_COMMANDS:

            if keyword in text:

                return ('follow_on', None)

        return None

    def _execute_direct_command(
        self,
        direct,
    ):

        kind, value = direct

        if kind == 'gesture':

            self._direct_gesture(
                value
            )

        elif kind == 'follow_off':

            self._direct_set_follow(
                False
            )

        elif kind == 'follow_on':

            self._direct_set_follow(
                True
            )

        elif kind == 'relock':

            self._direct_relock()

    def _begin_follow_command(self):
        with self._follow_cmd_lock:
            self._follow_cmd_generation += 1
            return self._follow_cmd_generation

    def _follow_command_current(self, generation):
        with self._follow_cmd_lock:
            return generation == self._follow_cmd_generation

    def _direct_set_follow(
        self,
        enable: bool,
    ):

        self.get_logger().info(
            f'🚶 直连跟随开关: '
            f'{"开启" if enable else "停止"}'
        )

        # 停掉当前正在播的声音再执行
        self._stop_current_output()

        generation = self._begin_follow_command()

        if enable:
            self._follow_requested = True
        else:
            self._follow_requested = False

        # 记录发指令的时间，只接受之后到达的锁定状态
        since = time.monotonic()
        self.bridge.set_follow(enable)

        if not enable:
            self.tts.speak('好的，我停下等你')
            return

        data = self._wait_for_lock(since, timeout=1.5)

        # 期间用户又说了"停止跟随"，不再播报旧结果
        if not self._follow_command_current(generation):
            return

        self.tts.speak(
            self._lock_feedback(data)
        )

    @staticmethod
    def _lock_feedback(data):
        if not data or not data.get('lock_enabled', False):
            return '好的，我跟上你了'

        state = data.get('lock_state')

        if state == 'LOCKED':
            return (
                '好的，我只跟着你'
                if data.get('detected')
                else '我看到你了，正在确认位置'
            )

        if state == 'PENDING':
            return '请站到我正前方，我看到你就跟上'

        if state == 'LOST':
            return '我正在找你，请站到我正前方'

        return '好的，我跟上你了'

    def _direct_relock(self):

        self.get_logger().info('🎯 直连重新锁定')
        self._stop_current_output()

        if not (
            self._follow_requested
            or self._follow_active
        ):
            self.tts.speak('请先对我说跟着我')
            return

        generation = self._begin_follow_command()
        since = time.monotonic()
        self.bridge.set_target_action('relock')
        data = self._wait_for_lock(since, timeout=1.5)

        if not self._follow_command_current(generation):
            return

        state = data.get('lock_state') if data else None

        if state == 'LOCKED':
            self.tts.speak('好的，我重新锁定你了')
        elif state in ('PENDING', 'LOST'):
            self.tts.speak('我正在重新找你，请站到正前方')
        else:
            self.tts.speak('重新锁定失败，请检查感知')

    def _direct_gesture(
        self,
        gesture_id: int,
    ):

        name = GESTURES.get(
            gesture_id,
            str(gesture_id),
        )

        self.get_logger().info(
            f'🚦 直连手势: {gesture_id}={name}'
        )

        self._stop_current_output()

        # ------------------------------------------------
        # 跟随中禁止手势：行走中摆手臂会破坏平衡直接倒
        # ------------------------------------------------

        if self._follow_active or self._follow_requested:

            self.tts.speak(
                '我正在跟随，'
                '请先说停止跟随，'
                '站稳后我再做手势'
            )

            return

        # ------------------------------------------------
        # 站立态(pd_stand)才做手势；walk 下做手势会倒。
        # 停跟随链路会自动 walk→pd_stand（约 1.5 秒），
        # 其他状态交给 MotionBridge 自动爬链。
        # ------------------------------------------------

        motion = (
            self.bridge.current_motion
        )

        if motion == 'walk':

            self.tts.speak(
                '我还没站稳，'
                '请先说停止跟随'
            )

            return

        # execute_tool('gesture') 内部轮询等服务响应
        # （最长 5 秒），绝不能堵麦克风线程，丢 worker
        threading.Thread(

            target=
            self.bridge.execute_tool,

            args=(
                'gesture',
                {'gesture_id': gesture_id},
            ),

            daemon=True,

        ).start()

        self.tts.speak(
            f'好的，{name}手势'
        )

    # ========================================================
    # 跟随侧安全桥状态回调
    #
    # status 是 JSON：
    # {"state":"FORWARDING","reason":"ok","armed":true,...}
    # ========================================================

    def _on_follow_status(
        self,
        msg,
    ):

        try:

            data = json.loads(
                msg.data
            )

            self._follow_active = (
                data.get('state') == 'FORWARDING'
            )

            # 语音发起的跟随即使目标暂时丢失（armed 仍为 true）
            # 也要算"跟随请求中"，动作守卫继续生效。
            if data.get('armed_source') == 'voice':

                self._follow_requested = bool(
                    data.get('armed', False)
                )

            elif not data.get('armed', False):

                self._follow_requested = False

        except Exception:

            # 解析不了就按"未跟随"处理，
            # 守卫放行，不影响原有动作链路
            self._follow_active = False

    # ========================================================
    # 单人锁定状态回调 / 等待
    # ========================================================

    def _on_person_status(
        self,
        msg,
    ):

        try:

            data = json.loads(
                msg.data
            )

        except Exception:

            return

        if not isinstance(data, dict):

            return

        with self._person_status_cond:

            self._person_status = data
            self._person_status_time = time.monotonic()
            self._person_status_cond.notify_all()

    def _wait_for_lock(
        self,
        since: float,
        timeout: float = 1.5,
    ):

        deadline = time.monotonic() + timeout

        with self._person_status_cond:

            while True:

                data = self._person_status

                if (
                    data
                    and self._person_status_time >= since
                    and data.get('lock_state')
                    in ('LOCKED', 'LEGACY')
                ):

                    return dict(data)

                remaining = deadline - time.monotonic()

                if remaining <= 0:

                    return dict(data) if data else {}

                self._person_status_cond.wait(
                    min(0.1, remaining)
                )

    # ========================================================
    # 普通 LLM 对话
    # ========================================================

    def _handle_utterance(
        self,
        text: str,
        turn_id: int,
    ):

        try:

            # =================================================
            # 1. LLM
            # =================================================

            reply, tool_calls = (
                self.llm.chat(
                    text
                )
            )

            # =================================================
            # 如果 LLM 思考过程中
            #
            # 用户重新说了：
            #
            # 你好小智
            #
            # 上一轮回答作废
            # =================================================

            if not self._is_current_turn(
                turn_id
            ):

                self.get_logger().info(
                    '🛑 上一轮 LLM 回答已作废'
                )

                return

            self.get_logger().info(
                f'💬 回复: {reply}'
            )

            # =================================================
            # 打印动作
            # =================================================

            for tc in tool_calls:

                self.get_logger().info(
                    f'🔧 调用工具: '
                    f'{tc["name"]} '
                    f'{tc["arguments"]}'
                )

            # =================================================
            # 2. 执行动作
            # =================================================

            for tc in tool_calls:

                if not (
                    self._is_current_turn(
                        turn_id
                    )
                ):

                    return

                if tc['name'] == 'play_media':
                    topic = str(tc['arguments'].get('topic', '')).strip()
                    if topic:
                        self._stop_current_output()
                        self._play_local_mp3(topic + '.mp3')
                    continue

                # ---------------------------------------------
                # 跟随模式守卫
                #
                # 跟随期间 LLM 的动作指令一律拦截：
                # - walk/switch_motion/wave_hand/gesture：
                #   本机行走由跟随模式负责，误触发会把
                #   状态切乱甚至切到 passive 瘫软
                # - emergency_stop：改为停跟随（保持站立），
                #   不切 passive
                # ---------------------------------------------

                if (
                    (
                        self._follow_active
                        or self._follow_requested
                    )
                    and tc['name']
                    in _FOLLOW_GUARDED_TOOLS
                ):

                    self.get_logger().info(
                        f'🛡 跟随中拦截动作指令: '
                        f'{tc["name"]}'
                    )

                    if (
                        tc['name']
                        == 'emergency_stop'
                    ):

                        # 停跟随（安全桥会补零速刹住），
                        # 不切 passive
                        self.bridge.execute_tool(
                            'set_follow',
                            {'enable': False},
                        )

                        # 急停不播报
                        return

                    self.tts.speak(
                        '我正在跟随模式，'
                        '如需手动控制，'
                        '请先说停止跟随'
                    )

                    return

                self.bridge.execute_tool(

                    tc['name'],

                    tc['arguments'],
                )

                # ------------------------------------------------
                # 急停不播报
                # ------------------------------------------------

                if (
                    tc['name']
                    == 'emergency_stop'
                ):

                    return

            # =================================================
            # 3. TTS
            # =================================================

            if (
                reply
                and
                self._is_current_turn(
                    turn_id
                )
            ):

                self.tts.speak(
                    reply
                )

        except Exception as e:

            self.get_logger().error(
                f'处理失败: '
                f'{type(e).__name__}: '
                f'{e}'
            )

    # ========================================================
    # 播放本地 MP3
    #
    # 这里直接使用你现有的 SpeakerPlayer
    #
    # 所以不需要 mpg123
    # ========================================================

    def _play_local_mp3(
        self,
        filename: str,
    ):

        path = os.path.join(

            self._get_media_dir(),

            filename,
        )

        if not os.path.isfile(
            path
        ):

            self.get_logger().error(
                f'❌ 内置MP3不存在: '
                f'{path}'
            )

            return

        # ----------------------------------------------------
        # 单独线程播放
        #
        # 不堵塞麦克风
        # ----------------------------------------------------

        threading.Thread(

            target=
            self._play_local_mp3_worker,

            args=(
                path,
            ),

            daemon=True,

        ).start()

    # ========================================================
    # MP3 播放线程
    # ========================================================

    def _play_local_mp3_worker(
        self,
        path: str,
    ):

        with self._media_lock:

            self._media_active = True

        try:

            # ------------------------------------------------
            # 读取 MP3 文件
            # ------------------------------------------------

            with open(
                path,
                'rb'
            ) as f:

                mp3_bytes = f.read()

            self.get_logger().info(
                f'🔊 开始播放内置MP3: '
                f'{path}'
            )

            # =================================================
            # 直接复用：
            #
            # TTSClient
            #      ↓
            # SpeakerPlayer
            #
            # 它内部已经会：
            #
            # MP3
            # ↓
            # ffmpeg
            # ↓
            # WAV
            # ↓
            # aplay
            # =================================================

            self.tts.player.play_mp3(
                mp3_bytes
            )

            self.get_logger().info(
                '✅ 内置MP3播放结束'
            )

        except Exception as e:

            # 被“你好小智”打断时
            # aplay 会被 kill
            # 这里出现提示属于正常现象

            self.get_logger().warning(
                f'内置MP3播放结束/被打断: '
                f'{e}'
            )

        finally:

            with self._media_lock:

                self._media_active = False

    # ========================================================
    # 停止当前声音
    #
    # 可以打断：
    #
    # LLM TTS
    # 内置 MP3
    # ========================================================

    def _stop_current_output(
        self
    ):

        try:

            # TTSClient.stop() 会置流式停止标志、kill 播放进程，
            # 并取消服务端剩余合成；SpeakerPlayer 同时也承载内置 MP3。
            # 只调 player.stop() 的话，流式 TTS 可能在打断后又被
            # 下一块音频重新启动。
            # ------------------------------------------------

            if hasattr(
                self.tts,
                'stop'
            ):

                self.tts.stop()

            elif hasattr(
                self.tts,
                'player'
            ):

                self.tts.player.stop()

        except Exception as e:

            self.get_logger().warning(
                f'停止当前声音失败: {e}'
            )

    # ========================================================
    # 当前是否正在播放声音
    # ========================================================

    def _is_output_active(
        self
    ) -> bool:

        # ----------------------------------------------------
        # TTS / aplay
        # ----------------------------------------------------

        try:

            if hasattr(
                self.tts,
                'player'
            ):

                if (
                    self.tts.player
                    .is_playing()
                ):

                    return True

        except Exception:

            pass

        # ----------------------------------------------------
        # 内置 MP3
        # ----------------------------------------------------

        with self._media_lock:

            return self._media_active

    # ========================================================
    # 获取 media 目录
    # ========================================================

    def _get_media_dir(
        self
    ) -> str:

        # ====================================================
        # 第一种
        #
        # voice_node.py 同级目录
        #
        # voice_interaction/
        # ├── voice_node.py
        # └── media/
        #     └── 1.mp3
        # ====================================================

        package_media = os.path.join(

            os.path.dirname(
                os.path.abspath(
                    __file__
                )
            ),

            'media',
        )

        if os.path.isdir(
            package_media
        ):

            return package_media

        # ====================================================
        # 第二种
        #
        # 兼容你现在这个测试工作空间
        # ====================================================

        fallback = os.path.join(
            os.environ.get(
                'PM01_WS',
                os.path.expanduser('~/pm01_ws'),
            ),
            'src',
            'voice_interaction',
            'voice_interaction',
            'media',
        )

        return fallback

    # ========================================================
    # 清理 ASR 文字
    # ========================================================

    @staticmethod
    def _normalize_text(
        text: str,
    ) -> str:

        if not text:

            return ''

        text = str(
            text
        ).strip()

        # ----------------------------------------------------
        # 去掉：
        #
        # 空格
        # 逗号
        # 句号
        # 问号
        # ...
        #
        # 例如：
        #
        # 你好，小志，跟着我。
        #
        # 变成：
        #
        # 你好小志跟着我
        # ----------------------------------------------------

        text = re.sub(

            r'[\s，,。.!！?？、；;：:"“”\'‘’]',

            '',

            text,
        )

        # ----------------------------------------------------
        # 把“小智”的同音字归一化成“小智”
        #
        # ASR 常把小智识别成：
        # 小志 / 小城 / 小程 / ...
        #
        # 唤醒判断有兼容列表没关系，
        # 但原文发给 LLM 会让模型
        # 纠结自己的名字（甚至自己编外号）。
        # ----------------------------------------------------

        text = re.sub(

            r'小[陈城程乘辰晨臣诚承呈橙]',

            '小智',

            text,
        )

        return text

    # ========================================================
    # 查找唤醒词
    # ========================================================

    @staticmethod
    def _find_wake_word(
        text: str,
    ):

        m = _WAKE_RE.search(text)

        return m.group(0) if m else None

    # ========================================================
    # 创建新一轮对话
    # ========================================================

    def _new_turn(
        self
    ) -> int:

        with self._turn_lock:

            self._turn_id += 1

            return self._turn_id

    # ========================================================
    # 判断是不是当前对话
    # ========================================================

    def _is_current_turn(
        self,
        turn_id: int,
    ) -> bool:

        with self._turn_lock:

            return (
                turn_id
                == self._turn_id
            )

    # ========================================================
    # 状态切换
    # ========================================================

    def _set_state(
        self,
        state: _State,
    ):

        with self._state_lock:

            self._state = state

    # ========================================================
    # 节点退出
    # ========================================================

    def destroy_node(
        self
    ):

        # ----------------------------------------------------
        # 停止声音
        # ----------------------------------------------------

        try:

            self._stop_current_output()

        except Exception:

            pass

        # ----------------------------------------------------
        # 停止 ASR
        # ----------------------------------------------------

        try:

            self.asr.stop()

        except Exception:

            pass

        # ----------------------------------------------------
        # 关闭麦克风
        # ----------------------------------------------------

        try:

            self.mic.close()

        except Exception:

            pass

        super().destroy_node()


# ============================================================
# main
# ============================================================

def main(
    args=None
):

    rclpy.init(
        args=args
    )

    node = (
        VoiceInteractionNode()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()