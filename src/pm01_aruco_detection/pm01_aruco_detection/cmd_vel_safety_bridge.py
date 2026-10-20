"""Armed, state-gated relay from visual commands to PM01 ``/cmd_vel``."""

import json
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

try:
    from interface_protocol.msg import MotionState
except ImportError:  # Optional on the camera computer.
    MotionState = None


class CmdVelSafetyBridge(Node):
    """Forward fresh commands only when explicitly armed and in the allowed state."""

    def __init__(self) -> None:
        super().__init__("pm01_cmd_vel_safety_bridge")
        self.declare_parameter("input_topic", "/vision_follow/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/vision_follow/safety_bridge_status")
        self.declare_parameter("motion_state_topic", "/motion/motion_state")
        self.declare_parameter("armed", False)
        self.declare_parameter("voice_control", True)
        self.declare_parameter("allowed_motion_state", "walk")
        self.declare_parameter("require_motion_state", True)
        self.declare_parameter("enable_forward", True)
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("motion_state_timeout_sec", 1.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("max_forward_mps", 0.08)
        self.declare_parameter("max_yaw_rps", 0.30)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        motion_state_topic = str(self.get_parameter("motion_state_topic").value)
        self.armed = bool(self.get_parameter("armed").value)
        # 运行时开关：语音 MotionBridge 通过 /vision_follow/set_follow
        # 远程 arm/disarm（"跟着我"/"停止跟随"）。armed_source 记录
        # 当前 armed 状态由谁设置，方便排查"谁在驱动机器人"。
        self.voice_control = bool(self.get_parameter("voice_control").value)
        self.armed_source = "launch"
        # disarm 后补发零速的时间窗（ros2_control 可保持末速，静默≠停稳）
        self._neutral_deadline = 0.0
        self.allowed_motion_state = str(self.get_parameter("allowed_motion_state").value)
        self.require_motion_state = bool(self.get_parameter("require_motion_state").value)
        self.enable_forward = bool(self.get_parameter("enable_forward").value)
        self.command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self.motion_state_timeout = float(self.get_parameter("motion_state_timeout_sec").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.max_forward = float(self.get_parameter("max_forward_mps").value)
        self.max_yaw = float(self.get_parameter("max_yaw_rps").value)
        if min(
            self.command_timeout,
            self.motion_state_timeout,
            publish_rate,
            self.max_forward,
            self.max_yaw,
        ) <= 0.0:
            raise ValueError("timeouts, publish rate, and limits must be positive")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.on_command, 10)
        if self.require_motion_state and MotionState is None:
            raise RuntimeError(
                "require_motion_state=true needs interface_protocol; install or source that package"
            )
        if MotionState is not None:
            self.create_subscription(MotionState, motion_state_topic, self.on_motion_state, sensor_qos)
        if self.voice_control:
            self.create_subscription(
                Bool, "/vision_follow/set_follow", self.on_set_follow, 10
            )
        self.last_command = Twist()
        self.last_command_time = None
        self.motion_state = "UNKNOWN"
        self.last_motion_state_time = None
        self.create_timer(1.0 / publish_rate, self.on_timer)
        self.get_logger().info(
            f"Safety bridge: {self.input_topic} -> {self.output_topic}, armed={self.armed}, "
            f"required_state={self.allowed_motion_state}, require_state={self.require_motion_state}"
        )

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def on_command(self, message: Twist) -> None:
        values = (message.linear.x, message.angular.z)
        if not all(math.isfinite(float(value)) for value in values):
            self.last_command = Twist()
            self.last_command_time = None
            return
        command = Twist()
        if self.enable_forward:
            command.linear.x = self.clamp(float(message.linear.x), 0.0, self.max_forward)
        command.angular.z = self.clamp(float(message.angular.z), -self.max_yaw, self.max_yaw)
        self.last_command = command
        self.last_command_time = time.monotonic()

    def on_motion_state(self, message: MotionState) -> None:
        self.motion_state = str(message.current_motion_task)
        self.last_motion_state_time = time.monotonic()

    def on_set_follow(self, message: Bool) -> None:
        if message.data:
            self.armed = True
            self.armed_source = "voice"
            self.get_logger().info("跟随开启（语音 set_follow=true）")
        else:
            self.armed = False
            self.armed_source = "voice"
            # 补 1 秒零速：on_timer 在 disarmed 时静默，
            # 不补脉冲的话 ros2_control 可能保持末速继续滑行
            self._neutral_deadline = time.monotonic() + 1.0
            self.get_logger().info("跟随停止（语音 set_follow=false），补零速 1s")

    def decision(self):
        now = time.monotonic()
        command_fresh = (
            self.last_command_time is not None
            and now - self.last_command_time <= self.command_timeout
        )
        state_fresh = (
            self.last_motion_state_time is not None
            and now - self.last_motion_state_time <= self.motion_state_timeout
        )
        state_allowed = (
            not self.require_motion_state
            or (state_fresh and self.motion_state == self.allowed_motion_state)
        )
        active = self.armed and command_fresh and state_allowed
        if not self.armed:
            reason = "disarmed"
        elif not command_fresh:
            reason = "command_timeout"
        elif not state_fresh and self.require_motion_state:
            reason = "motion_state_timeout"
        elif not state_allowed:
            reason = f"motion_state={self.motion_state}"
        else:
            reason = "ok"
        return active, reason

    def on_timer(self) -> None:
        active, reason = self.decision()
        output = self.last_command if active else Twist()
        if self.armed:
            self.cmd_pub.publish(output)
        elif time.monotonic() < self._neutral_deadline:
            # disarm 后的零速脉冲窗口，确保机器人刹住
            self.cmd_pub.publish(Twist())
        status = String()
        status.data = json.dumps(
            {
                "state": "FORWARDING" if active else "BLOCKED",
                "reason": reason,
                "armed": self.armed,
                "armed_source": self.armed_source,
                "motion_state": self.motion_state,
                "allowed_motion_state": self.allowed_motion_state,
                "linear_mps": round(float(output.linear.x), 4),
                "angular_rps": round(float(output.angular.z), 4),
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(status)

    def publish_neutral(self) -> None:
        if self.armed:
            self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelSafetyBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_neutral()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
