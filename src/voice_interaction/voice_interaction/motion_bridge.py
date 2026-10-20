"""ROS2 动作桥：把 LLM Function Call 转换成 PM01 ROS2 控制指令。

主要功能：
1. walk            行走 / 平移 / 转向 / 停止
2. switch_motion   切换 PM01 运动状态
3. wave_hand       左手 / 右手挥手
4. gesture         交警手势（/motion/gesture 服务）
5. emergency_stop  紧急停止

关键安全逻辑：
- PM01 状态机不允许跳跃（真机已验证）：
  idle → passive → pd_stand → rl_basic → body_vel_cmd
- 非行走状态下收到 walk，不立即发送速度，
  沿链逐级请求切换，每级等 /motion/motion_state 确认后再走下一级，
  到达行走状态后才发送速度。
- 普通停止只发送 0 速度，不切 passive。
- emergency_stop 才发送 0 速度并切 passive。
"""

import threading
import time

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from interface_protocol.msg import (
    BodyVelCmd,
    MotionStateRequest,
    MotionState,
    JointOverrideCommand,
)
from interface_protocol.srv import MotionGesture
from std_msgs.msg import Bool, Header


# ============================================================
# PM01 手臂关节配置
# ============================================================

# 右臂
# J18_SHOULDER_PITCH_R
# J19_SHOULDER_ROLL_R
# J21_ELBOW_PITCH_R
_RIGHT_ARM_WAVE = {
    'indices': [18, 19, 21],
    'raise': [-1.4, 0.6, -0.6],
    'stiffness': [50.0, 50.0, 50.0],
    'damping': [1.0, 1.0, 1.0],
}

# 左臂
_LEFT_ARM_WAVE = {
    'indices': [13, 14, 16],
    'raise': [-1.4, -0.6, -0.6],
    'stiffness': [50.0, 50.0, 50.0],
    'damping': [1.0, 1.0, 1.0],
}


# PM01 可以接受 BodyVelCmd 的行走状态
# （'walk' 是第二套开发 SDK 的行走状态名，2026-08-31 从
#   运控固件二进制 strings 实测抠出；不加的话跟随/行走中
#   再说"向前走"会把 walk 当链外状态请求切 passive 瘫软）
_WALK_STATES = {
    'rl_basic',
    'rl_amp',
    'rl_terrain',
    'walk',
}


# ------------------------------------------------------------
# PM01 状态机不允许跳跃（真机已验证）：
#
# idle → passive → pd_stand → rl_basic → body_vel_cmd
#
# 收到 walk 而当前不在行走状态时，
# 沿这条链逐级请求切换，
# 每一级都等 /motion/motion_state 确认后再走下一级。
# ------------------------------------------------------------

_WALK_LADDER = [
    'idle',
    'passive',
    'pd_stand',
    'rl_basic',
]

# 每一级切换的等待秒数
# （状态一有变化就重新计时，起立可能要好几秒）
_WALK_STEP_TIMEOUT = 8.0

# 同一跳请求的重发间隔
# （状态一直没变化时定期补发，
#   防止单条 MotionStateRequest 丢失卡死在当前级）
_HOP_RETRY_INTERVAL = 2.0

# ------------------------------------------------------------
# 停止跟随后等几秒再切 pd_stand：
# 先让安全桥的零速脉冲把机器人停稳，行走中直接切站立会前倾
# ------------------------------------------------------------

_FOLLOW_STOP_SETTLE_SEC = 1.5


# ------------------------------------------------------------
# 第一阶段真机调试限速（知识库建议：先从小速度开始测试）
# 现场觉得还是快/慢，只改这三个数即可
# ------------------------------------------------------------

# 前进/后退 m/s（原 0.30，用户反馈偏快，降一半）
_MAX_VEL_X = 0.15

# 左右平移 m/s（原 0.20）
_MAX_VEL_Y = 0.10

# 转向角速度 rad/s（原 0.40）
_MAX_VEL_YAW = 0.30


# ------------------------------------------------------------
# 交警手势（运控单元 /motion/gesture 服务）
# ------------------------------------------------------------

GESTURES = {
    0: '归位',
    1: '停止',
    2: '直行',
    3: '左转弯',
    4: '左转弯待转',
    5: '右转弯',
    6: '变道',
    7: '减速慢行',
    8: '靠边停车',
}

# 手势服务响应等待秒数
_GESTURE_TIMEOUT = 5.0


def _next_walk_hop(current: str):
    """返回从当前状态通往行走状态的下一跳。

    已在行走状态返回 None；
    不在链上的状态（未知 / 旁支如 joint_bridge）
    先请求回到 passive 再爬链。
    """
    if current in _WALK_STATES:
        return None

    if current in _WALK_LADDER:
        return _WALK_LADDER[
            _WALK_LADDER.index(current) + 1
        ]

    return 'passive'


class MotionBridge:
    """PM01 动作执行器。"""

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

        # ====================================================
        # ROS2 Publisher
        # ====================================================

        # 机体速度控制
        self._vel_pub = node.create_publisher(
            BodyVelCmd,
            '/motion/body_vel_cmd',
            10,
        )

        # 运动状态切换
        self._state_pub = node.create_publisher(
            MotionStateRequest,
            '/motion/set_motion_state',
            10,
        )

        # 上肢关节覆盖控制
        self._override_pub = node.create_publisher(
            JointOverrideCommand,
            '/motion/joint_override_command',
            10,
        )

        # 视觉跟随开关（跟随侧安全桥订阅，
        # True=开始跟随 False=停止跟随并零速停稳）
        self._follow_pub = node.create_publisher(
            Bool,
            '/vision_follow/set_follow',
            10,
        )

        # 交警手势服务客户端
        self._gesture_client = node.create_client(
            MotionGesture,
            '/motion/gesture',
        )

        # ====================================================
        # ROS2 Subscriber
        # ====================================================

        # 这套运控 SDK 的 /motion/motion_state 是
        # BEST_EFFORT 发布（跟随侧安全桥实测同样结论），
        # 默认 RELIABLE 订阅会静默失配一条都收不到，
        # 站立化/爬链全瞎。必须显式 BEST_EFFORT。
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._motion_state_sub = node.create_subscription(
            MotionState,
            '/motion/motion_state',
            self._on_motion_state,
            sensor_qos,
        )

        # ====================================================
        # 当前机器人运动状态
        # ====================================================

        self._current_motion = ''
        self._available_transitions = []

        # ----------------------------------------------------
        # 停跟随后的站立化请求
        #
        # walk 下直接做手势会倒（手臂运动会破坏行走的
        # 平衡控制），所以"停止跟随"要做两件事：
        #   1. 安全桥 disarm + 零速脉冲（速度先停）
        #   2. 等机器人停稳（STOP_SETTLE 秒）再切 pd_stand
        # 站立化放 worker 线程（sleep 不堵主线程），
        # _stand_request_id 用来让过期请求作废
        # （快速连喊"停/走"时只认最新一条）。
        # ----------------------------------------------------
        self._stand_request_id = 0

        # ====================================================
        # 行走控制
        # ====================================================

        # 当前持续发送的速度
        # None            = 当前没有持续行走
        # (lx, ly, yaw)   = 当前行走速度
        self._current_vel = None

        self._vel_timer = None

        self._vel_lock = threading.Lock()

        # ----------------------------------------------------
        # 等待状态切换后执行的 walk
        # ----------------------------------------------------
        #
        # 例如：
        #
        # 用户说：“向前走”
        #
        # 当前：
        # pd_stand
        #
        # 不立即发送 0.3m/s
        #
        # 而是：
        #
        # pending_walk = (0.3, 0, 0)
        #
        # 请求：
        # rl_basic
        #
        # 等 motion_state 确认 rl_basic 后
        # 再真正发送速度。
        #

        # pending_walk 这组状态被 executor 回调(_on_motion_state)和
        # worker 线程(_do_walk)同时读写，用 RLock 保证整组更新原子。
        # 可重入：_check_pending_walk 内部会再调 _advance_walk_chain。
        self._walk_lock = threading.RLock()

        self._pending_walk = None
        self._pending_walk_deadline = 0.0

        # ----------------------------------------------------
        # 自动切链进度：
        #
        # _walk_last_request = (发起请求时的状态, 目标状态)
        #   相同则不重复发请求（motion_state 高频回调）
        #
        # _walk_last_seen = 上次见到的运动状态
        #   状态一变化视为切链有进展，刷新超时
        # ----------------------------------------------------

        self._walk_last_request = None

        self._walk_last_request_time = 0.0

        self._walk_last_seen = None

        # ====================================================
        # 挥手
        # ====================================================

        self._wave_timer = None

        # 等待 lower_body_balance 后执行的挥手
        self._pending_wave = None
        self._pending_wave_deadline = 0.0

        self.logger.info('MotionBridge 初始化完成')

    # ========================================================
    # MotionState 回调
    # ========================================================

    def _on_motion_state(self, msg: MotionState):
        """接收 PM01 当前运动状态。"""

        old_motion = self._current_motion

        self._current_motion = msg.current_motion_task

        self._available_transitions = list(
            msg.available_transition_motions
        )

        # 状态真正变化时才打印
        if old_motion != self._current_motion:
            self.logger.info(
                f'🤖 运动状态: '
                f'{old_motion or "未知"} '
                f'→ {self._current_motion}'
            )

            self.logger.info(
                f'🔀 当前允许切换: '
                f'{self._available_transitions}'
            )

        # 检查有没有等待中的行走命令
        self._check_pending_walk()

        # 检查有没有等待中的挥手命令
        self._check_pending_wave()

    @property
    def current_motion(self) -> str:
        return self._current_motion

    @property
    def available_transitions(self):
        return list(self._available_transitions)

    # ========================================================
    # LLM Tool 统一入口
    # ========================================================

    def execute_tool(self, name: str, args: dict):
        """执行 Qwen Function Call。"""

        try:
            if name == 'walk':
                self._do_walk(args)

            elif name == 'switch_motion':
                self._do_switch_motion(args)

            elif name == 'wave_hand':
                self._do_wave(args)

            elif name == 'gesture':
                self._do_gesture(args)

            elif name == 'emergency_stop':
                self._do_emergency_stop()

            elif name == 'set_follow':
                self._do_set_follow(args)

            else:
                self.logger.warning(
                    f'未知工具: {name}'
                )

        except Exception as e:
            self.logger.error(
                f'执行工具 {name} 失败: '
                f'{type(e).__name__}: {e}'
            )

    # ========================================================
    # WALK
    # ========================================================

    def _do_walk(self, args: dict):
        """处理行走命令。"""

        lx = float(
            args.get('linear_x', 0.0)
        )

        ly = float(
            args.get('linear_y', 0.0)
        )

        yaw = float(
            args.get('yaw', 0.0)
        )

        # ----------------------------------------------------
        # 第一阶段真机调试限速
        # ----------------------------------------------------
        #
        # LLM 给多大会被压到 _MAX_VEL_* 以内，
        # 具体数值见文件顶部的限速常量。
        #

        lx = max(
            -_MAX_VEL_X,
            min(_MAX_VEL_X, lx),
        )

        ly = max(
            -_MAX_VEL_Y,
            min(_MAX_VEL_Y, ly),
        )

        yaw = max(
            -_MAX_VEL_YAW,
            min(_MAX_VEL_YAW, yaw),
        )

        # ----------------------------------------------------
        # 判断是否是停止命令
        # ----------------------------------------------------

        is_stop = (
            abs(lx) < 1e-6
            and abs(ly) < 1e-6
            and abs(yaw) < 1e-6
        )

        # ====================================================
        # STOP
        # ====================================================

        if is_stop:
            self._clear_pending_walk()

            with self._vel_lock:
                self._current_vel = None

            self._stop_vel_timer()

            # 连续发几次 0，增加停止可靠性
            self._publish_vel(
                0.0,
                0.0,
                0.0,
            )

            self.logger.info(
                '■ 停止行走'
            )

            return

        # ====================================================
        # 已经处于行走状态
        # ====================================================

        if self._current_motion in _WALK_STATES:
            self._start_walk(
                lx,
                ly,
                yaw,
            )

            return

        # ====================================================
        # 不在行走状态
        #
        # PM01 状态机不允许跳跃，
        # 挂起速度命令，沿链自动逐级切换：
        #
        # 当前状态 → passive → pd_stand → rl_basic → 发速度
        #
        # 每一级由 /motion/motion_state 回调确认后推进。
        # ====================================================

        self._set_pending_walk(lx, ly, yaw)

        self.logger.info(
            f'⏳ 当前状态 '
            f'{self._current_motion or "未知"}，'
            f'开始自动切换到行走状态'
        )

        self._advance_walk_chain()

    def _set_pending_walk(self, lx, ly, yaw):
        """挂起一条行走命令，等切到行走状态后再发速度（整组状态在锁内更新）。"""
        with self._walk_lock:
            self._pending_walk = (lx, ly, yaw)
            self._pending_walk_deadline = (
                time.monotonic() + _WALK_STEP_TIMEOUT
            )
            self._walk_last_request = None
            self._walk_last_seen = self._current_motion

    def _check_pending_walk(self):
        """MotionState 更新后推进等待中的 walk（自动切链主驱动）。"""
        with self._walk_lock:
            return self._check_pending_walk_impl()

    def _check_pending_walk_impl(self):
        if self._pending_walk is None:
            return

        cur = self._current_motion

        # ----------------------------------------------------
        # 状态有变化 = 切链有进展，顺延超时
        # （起立 pd_stand 可能需要好几秒）
        # ----------------------------------------------------

        if cur != self._walk_last_seen:

            self._walk_last_seen = cur

            self._pending_walk_deadline = (
                time.monotonic()
                + _WALK_STEP_TIMEOUT
            )

        elif (
            time.monotonic()
            > self._pending_walk_deadline
        ):
            self.logger.warning(
                f'⚠ 自动切换到行走状态超时'
                f'（卡在 {cur or "未知"}），'
                f'取消本次行走命令'
            )

            self._clear_pending_walk()

            return

        # ----------------------------------------------------
        # 到达行走状态，执行挂起的速度命令
        # ----------------------------------------------------

        if cur in _WALK_STATES:
            lx, ly, yaw = self._pending_walk

            self._clear_pending_walk()

            self.logger.info(
                f'✅ 已进入 {cur}，'
                f'开始执行等待中的行走命令'
            )

            self._start_walk(
                lx,
                ly,
                yaw,
            )

            return

        # 还没到行走状态，继续请求下一跳
        self._advance_walk_chain()

    def _advance_walk_chain(self):
        """沿状态链请求下一跳，直到进入行走状态。

        idle → passive → pd_stand → rl_basic
        """
        with self._walk_lock:
            return self._advance_walk_chain_impl()

    def _advance_walk_chain_impl(self):
        cur = self._current_motion

        hop = _next_walk_hop(cur)

        if hop is None:
            return

        # ----------------------------------------------------
        # 同一状态下的同一目标不重复刷屏，
        # 但超过重发间隔且状态一直没变化时补发一次，
        # 防止单条请求丢失卡死在当前级
        # ----------------------------------------------------

        now = time.monotonic()

        if (
            self._walk_last_request
            == (cur, hop)
        ):
            if (
                now
                - self._walk_last_request_time
                < _HOP_RETRY_INTERVAL
            ):
                return

            self._publish_switch_motion(hop)

            self._walk_last_request_time = now

            self.logger.info(
                f'↻ 状态未变，重发请求: {hop}'
            )

            return

        # ----------------------------------------------------
        # 运控明确告知不可达时放弃，不硬切
        # ----------------------------------------------------

        if (
            cur
            and self._available_transitions
            and hop
            not in self._available_transitions
        ):
            self.logger.warning(
                f'⚠ 当前状态 {cur} '
                f'不能切换到 {hop}，'
                f'取消本次行走命令'
            )

            self.logger.warning(
                f'当前允许切换: '
                f'{self._available_transitions}'
            )

            self._clear_pending_walk()

            return

        self._publish_switch_motion(hop)

        self._walk_last_request = (
            cur,
            hop,
        )

        self._walk_last_request_time = now

        self.logger.info(
            f'▶ 自动切链: '
            f'{cur or "未知"} → {hop}'
        )

    def _clear_pending_walk(self):
        """清空切链进度。"""
        with self._walk_lock:
            return self._clear_pending_walk_impl()

    def _clear_pending_walk_impl(self):
        self._pending_walk = None

        self._walk_last_request = None

        self._walk_last_request_time = 0.0

        self._walk_last_seen = None

    def _start_walk(
        self,
        lx: float,
        ly: float,
        yaw: float,
    ):
        """真正开始持续发送 BodyVelCmd。"""

        with self._vel_lock:
            self._current_vel = (
                lx,
                ly,
                yaw,
            )

        # 马上发一次
        self._publish_vel(
            lx,
            ly,
            yaw,
        )

        # 后续持续发送
        self._start_vel_timer()

        self.logger.info(
            f'🚶 真正开始行走 '
            f'lx={lx:.2f} '
            f'ly={ly:.2f} '
            f'yaw={yaw:.2f}'
        )

    # ========================================================
    # BodyVelCmd 持续发布
    # ========================================================

    def _start_vel_timer(self):
        """100Hz 持续发送速度命令。"""

        if self._vel_timer is None:
            self._vel_timer = (
                self.node.create_timer(
                    0.01,
                    self._vel_timer_cb,
                )
            )

    def _stop_vel_timer(self):
        if self._vel_timer is not None:
            try:
                self._vel_timer.cancel()

                self.node.destroy_timer(
                    self._vel_timer
                )

            except Exception:
                pass

            self._vel_timer = None

    def _vel_timer_cb(self):
        """持续发送当前速度。"""

        with self._vel_lock:
            current_vel = self._current_vel

        if current_vel is None:
            return

        lx, ly, yaw = current_vel

        self._publish_vel(
            lx,
            ly,
            yaw,
        )

    def _publish_vel(
        self,
        lx: float,
        ly: float,
        yaw: float,
    ):
        """发布 PM01 BodyVelCmd。"""

        msg = BodyVelCmd()

        msg.header = self._make_header(
            'body'
        )

        msg.linear_velocity = [
            float(lx),
            float(ly),
        ]

        msg.yaw_velocity = float(yaw)

        self._vel_pub.publish(
            msg
        )

    # ========================================================
    # SWITCH MOTION
    # ========================================================

    def _do_switch_motion(self, args: dict):
        """LLM 请求切换运动状态。"""

        state = str(
            args.get(
                'state',
                'passive',
            )
        )

        # 当前已经处于目标状态
        if self._current_motion == state:
            self.logger.info(
                f'当前已经处于 {state}，'
                f'无需重复切换'
            )

            return

        # ----------------------------------------------------
        # 如果 motion_state 已经告诉我们允许的转换，
        # 就严格遵守 available_transition_motions。
        # ----------------------------------------------------

        if (
            self._current_motion
            and self._available_transitions
            and state
            not in self._available_transitions
        ):
            self.logger.warning(
                f'⚠ 无法从 '
                f'{self._current_motion} '
                f'直接切换到 {state}'
            )

            self.logger.warning(
                f'当前允许切换: '
                f'{self._available_transitions}'
            )

            return

        self._publish_switch_motion(
            state
        )

        self.logger.info(
            f'▶ 请求切换运动状态 → '
            f'{state}'
        )

    def _publish_switch_motion(
        self,
        state: str,
    ):
        """发布 MotionStateRequest。"""

        msg = MotionStateRequest()

        msg.target_motion_name = state

        self._state_pub.publish(
            msg
        )

    # ========================================================
    # WAVE HAND
    # ========================================================

    def _do_wave(self, args: dict):
        """挥手。"""

        side = str(
            args.get(
                'side',
                'right',
            )
        ).lower()

        if side not in {
            'left',
            'right',
            'both',
        }:
            side = 'right'

        # ----------------------------------------------------
        # 上肢覆盖控制需要 lower_body_balance
        # ----------------------------------------------------

        if (
            self._current_motion
            != 'lower_body_balance'
        ):
            self._pending_wave = side

            self._pending_wave_deadline = (
                time.monotonic() + 5.0
            )

            self.logger.info(
                f'⏳ 挥手需要 '
                f'lower_body_balance，'
                f'当前状态: '
                f'{self._current_motion or "未知"}'
            )

            if (
                self._current_motion
                and self._available_transitions
                and 'lower_body_balance'
                not in self._available_transitions
            ):
                self.logger.warning(
                    '⚠ 当前状态无法直接切换到 '
                    'lower_body_balance'
                )

                self.logger.warning(
                    f'当前允许切换: '
                    f'{self._available_transitions}'
                )

                self._pending_wave = None

                return

            self._publish_switch_motion(
                'lower_body_balance'
            )

            self.logger.info(
                '▶ 已请求切换 '
                '→ lower_body_balance'
            )

            return

        # 已经在 lower_body_balance
        self._start_wave(
            side
        )

    def _check_pending_wave(self):
        """进入 lower_body_balance 后再真正挥手。"""

        if self._pending_wave is None:
            return

        if (
            time.monotonic()
            > self._pending_wave_deadline
        ):
            self.logger.warning(
                '⚠ 等待 lower_body_balance '
                '超时，取消挥手'
            )

            self._pending_wave = None

            return

        if (
            self._current_motion
            == 'lower_body_balance'
        ):
            side = self._pending_wave

            self._pending_wave = None

            self.logger.info(
                '✅ 已进入 lower_body_balance，'
                '开始挥手'
            )

            self._start_wave(
                side
            )

    def _start_wave(self, side: str):
        """真正发布 JointOverrideCommand。"""

        arms = []

        if side in (
            'right',
            'both',
        ):
            arms.append(
                _RIGHT_ARM_WAVE
            )

        if side in (
            'left',
            'both',
        ):
            arms.append(
                _LEFT_ARM_WAVE
            )

        for arm in arms:
            self._publish_override(
                arm['indices'],
                arm['raise'],
                arm['stiffness'],
                arm['damping'],
                weight=1.0,
            )

        self.logger.info(
            f'👋 开始挥手: {side}'
        )

        # 取消旧 timer
        if self._wave_timer is not None:
            try:
                self._wave_timer.cancel()

                self.node.destroy_timer(
                    self._wave_timer
                )

            except Exception:
                pass

            self._wave_timer = None

        # 3 秒后解除上肢覆盖
        self._wave_timer = (
            self.node.create_timer(
                3.0,
                self._clear_wave,
            )
        )

    def _clear_wave(self):
        """解除关节覆盖。"""

        arms = [
            _RIGHT_ARM_WAVE,
            _LEFT_ARM_WAVE,
        ]

        for arm in arms:
            self._publish_override(
                arm['indices'],
                [0.0]
                * len(
                    arm['indices']
                ),
                arm['stiffness'],
                arm['damping'],
                weight=0.0,
            )

        if self._wave_timer is not None:
            try:
                self._wave_timer.cancel()

                self.node.destroy_timer(
                    self._wave_timer
                )

            except Exception:
                pass

            self._wave_timer = None

        self.logger.info(
            '■ 挥手结束'
        )

    def _publish_override(
        self,
        indices,
        position,
        stiffness,
        damping,
        weight=1.0,
    ):
        """发布上肢 JointOverrideCommand。"""

        msg = JointOverrideCommand()

        msg.header = self._make_header(
            'body'
        )

        msg.weight = float(
            weight
        )

        msg.joint_indices = list(
            indices
        )

        msg.position = list(
            position
        )

        msg.velocity = [
            0.0
        ] * len(
            indices
        )

        msg.feed_forward_torque = [
            0.0
        ] * len(
            indices
        )

        msg.torque = [
            0.0
        ] * len(
            indices
        )

        msg.stiffness = list(
            stiffness
        )

        msg.damping = list(
            damping
        )

        self._override_pub.publish(
            msg
        )

    # ========================================================
    # EMERGENCY STOP
    # ========================================================

    def _do_emergency_stop(self):
        """紧急停止。

        与普通“停下”不同：
        普通停止 -> 速度归零
        紧急停止 -> 速度归零 + passive
        """

        # 取消所有等待动作
        self._clear_pending_walk()
        self._pending_wave = None

        # 停止持续速度
        with self._vel_lock:
            self._current_vel = None

        self._stop_vel_timer()

        # 先发送 0 速度
        self._publish_vel(
            0.0,
            0.0,
            0.0,
        )

        # 再切 passive
        self._publish_switch_motion(
            'passive'
        )

        self.logger.warning(
            '⚠ 紧急停止：'
            '速度归零 → passive'
        )

    # ========================================================
    # SET_FOLLOW 视觉跟随开关
    # ========================================================

    def set_follow(self, enable: bool):
        """开始/停止视觉跟随（公开接口）。

        True  -> 安全桥 armed，跟随指令放行到 /cmd_vel
                 （若机器人正被切往 pd_stand，一并撤销）
        False -> 安全桥 disarm + 零速脉冲，等停稳后
                 切 pd_stand 站立（walk 下做手势会倒，
                 必须回到站立态）

        直连指令（voice_node 命令表）和 LLM 工具调用都走这里。
        本方法只做发布和状态请求，秒级返回，可安全调用。
        """

        if enable:
            # 撤销还没执行的站立化请求
            self._stand_request_id += 1

            # 停跟随时机器人被切到 pd_stand，
            # 重新跟随必须先回 walk（pd_stand 下
            # 速度指令无效，机器人不会迈步）
            if (
                self._current_motion
                not in _WALK_STATES
            ):

                self._publish_switch_motion(
                    'walk'
                )

                self.logger.info(
                    '🚶 重新跟随: '
                    f'{self._current_motion or "未知"} '
                    '→ 请求切回 walk'
                )

        msg = Bool()
        msg.data = bool(enable)

        self._follow_pub.publish(
            msg
        )

        self.logger.info(
            '🚶 视觉跟随: '
            f'{"开启" if enable else "停止"}'
        )

        if not enable:
            self._stand_after_stop()

    def _do_set_follow(self, args: dict):
        """LLM 工具入口，转公开接口。"""

        self.set_follow(
            bool(
                args.get('enable', False)
            )
        )

    def _stand_after_stop(self):
        """停跟随后等停稳再切 pd_stand（worker 线程）。"""

        request_id = (
            self._stand_request_id + 1
        )
        self._stand_request_id = request_id

        def _worker():

            time.sleep(
                _FOLLOW_STOP_SETTLE_SEC
            )

            # 期间用户又喊"跟着我"（id 已变）则放弃
            if (
                self._stand_request_id
                != request_id
            ):

                self.logger.info(
                    '🧍 停跟随站立化: '
                    '已被新的跟随指令作废'
                )

                return

            current = (
                self._current_motion
                or '未知'
            )

            if current == 'pd_stand':

                self.logger.info(
                    '🧍 停跟随: 已是站立态，'
                    '无需切换'
                )

                return

            # 不论当前是什么状态（含未知）都发 pd_stand：
            # walk→pd_stand 是想要的转换；其他状态发了
            # 顶多被固件拒，总比静默不切强
            self._publish_switch_motion(
                'pd_stand'
            )

            self.logger.info(
                f'🧍 停跟随: {current} '
                '→ 已请求 pd_stand 站立'
            )

        threading.Thread(
            target=_worker,
            daemon=True,
        ).start()

    # ========================================================
    # GESTURE
    # ========================================================

    def _do_gesture(self, args: dict):
        """交警手势：通过 /motion/gesture 服务执行。"""

        try:
            gesture_id = int(
                args.get(
                    'gesture_id',
                    -1,
                )
            )

        except (TypeError, ValueError):

            gesture_id = -1

        if gesture_id not in GESTURES:

            self.logger.warning(
                f'⚠ 无效手势ID: '
                f'{gesture_id}，'
                f'可用: '
                f'{sorted(GESTURES)}'
            )

            return

        # ----------------------------------------------------
        # 先停行走（与 _do_walk 停止分支一致）：
        # 手势期间 BodyVelCmd 100Hz 速度流会和手势动作打架，
        # 先发零速并停定时器，再调用手势服务。
        # ----------------------------------------------------

        self._clear_pending_walk()

        with self._vel_lock:
            self._current_vel = None

        self._stop_vel_timer()

        self._publish_vel(
            0.0,
            0.0,
            0.0,
        )

        # 让零速落地，避免手势开始时速度流还没生效
        time.sleep(0.3)

        if not self._gesture_client.wait_for_service(
            timeout_sec=2.0
        ):

            self.logger.error(
                '❌ 手势服务 '
                '/motion/gesture 不可用'
            )

            return

        req = MotionGesture.Request()

        req.gesture_id = gesture_id

        future = (
            self._gesture_client
            .call_async(req)
        )

        deadline = (
            time.monotonic()
            + _GESTURE_TIMEOUT
        )

        # execute_tool 运行在 worker 线程，
        # 轮询等待服务响应（带超时，不阻塞主线程）
        while (
            rclpy.ok()
            and not future.done()
            and time.monotonic()
            < deadline
        ):

            time.sleep(0.05)

        if not future.done():

            self.logger.warning(
                '⚠ 手势服务响应超时'
            )

            return

        resp = future.result()

        if resp.success:

            self.logger.info(
                f'🚦 手势执行成功: '
                f'{gesture_id}='
                f'{GESTURES[gesture_id]}'
            )

        else:

            self.logger.warning(
                f'⚠ 手势执行失败: '
                f'{resp.message}'
            )

    # ========================================================
    # Header
    # ========================================================

    def _make_header(
        self,
        frame='body',
    ) -> Header:
        h = Header()

        h.stamp = (
            self.node
            .get_clock()
            .now()
            .to_msg()
        )

        h.frame_id = frame

        return h