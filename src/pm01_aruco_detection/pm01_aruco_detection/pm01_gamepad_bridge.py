"""Bridge visual follow Twist commands to the PM01 virtual gamepad LCM channel."""

import json
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

try:
    import lcm
except ImportError as exc:  # pragma: no cover - exercised only on missing runtime dep
    lcm = None
    _LCM_IMPORT_ERROR = exc
else:
    _LCM_IMPORT_ERROR = None

from .pm01_lcm import GamepadKeys
from .pm01_gamepad_mapping import shape_stick


class Pm01GamepadBridge(Node):
    """Publish PM01 virtual gamepad sticks from /vision_follow/cmd_vel."""

    def __init__(self) -> None:
        super().__init__("pm01_visual_follow_gamepad_bridge")
        self.declare_parameter("input_topic", "/vision_follow/cmd_vel")
        self.declare_parameter("status_topic", "/vision_follow/pm01_gamepad_bridge_status")
        self.declare_parameter("lcm_url", "udpm://239.255.76.67:7667?ttl=0")
        self.declare_parameter("channel", "virtual_gamepad/gamepad_keys")
        self.declare_parameter("armed", False)
        self.declare_parameter("enable_forward", True)
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("forward_scale", 0.4)
        self.declare_parameter("lateral_scale", 0.2)
        self.declare_parameter("yaw_scale", 0.45)
        self.declare_parameter("linear_command_deadband", 0.01)
        self.declare_parameter("yaw_command_deadband", 0.04)
        self.declare_parameter("minimum_linear_stick", 0.35)
        self.declare_parameter("minimum_yaw_stick", 0.25)

        if lcm is None:
            raise RuntimeError(f"python lcm module unavailable: {_LCM_IMPORT_ERROR}")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.lcm_url = str(self.get_parameter("lcm_url").value)
        self.channel = str(self.get_parameter("channel").value)
        self.armed = bool(self.get_parameter("armed").value)
        self.enable_forward = bool(self.get_parameter("enable_forward").value)
        self.timeout = float(self.get_parameter("command_timeout_sec").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.forward_scale = float(self.get_parameter("forward_scale").value)
        self.lateral_scale = float(self.get_parameter("lateral_scale").value)
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.linear_deadband = float(self.get_parameter("linear_command_deadband").value)
        self.yaw_deadband = float(self.get_parameter("yaw_command_deadband").value)
        self.minimum_linear_stick = float(self.get_parameter("minimum_linear_stick").value)
        self.minimum_yaw_stick = float(self.get_parameter("minimum_yaw_stick").value)
        if self.timeout <= 0.0 or publish_rate <= 0.0:
            raise ValueError("timeout and publish_rate_hz must be positive")

        self.lcm_handle = lcm.LCM(self.lcm_url)
        self.last_command = Twist()
        self.last_command_time = None
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.on_twist, 10)
        self.create_timer(1.0 / publish_rate, self.on_timer)
        self.get_logger().info(
            f"PM01 gamepad bridge ready: {self.input_topic} -> {self.channel}, "
            f"armed={self.armed}, enable_forward={self.enable_forward}, "
            f"scales fwd/yaw={self.forward_scale:.2f}/{self.yaw_scale:.2f}"
        )

    @staticmethod
    def _finite(message: Twist) -> bool:
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        return all(math.isfinite(float(value)) for value in values)

    def on_twist(self, message: Twist) -> None:
        if not self._finite(message):
            self.last_command = Twist()
            self.last_command_time = None
            return
        self.last_command = message
        self.last_command_time = time.monotonic()

    def on_timer(self) -> None:
        fresh = (
            self.last_command_time is not None
            and time.monotonic() - self.last_command_time <= self.timeout
        )
        active = self.armed and fresh
        forward = 0.0
        lateral = 0.0
        yaw = 0.0
        if active:
            message = GamepadKeys()
            message.timestamp = int(time.time_ns() / 1000)
            forward = (
                shape_stick(
                    float(self.last_command.linear.x),
                    self.forward_scale,
                    self.linear_deadband,
                    self.minimum_linear_stick,
                )
                if self.enable_forward
                else 0.0
            )
            lateral = shape_stick(
                -float(self.last_command.linear.y),
                self.lateral_scale,
                self.linear_deadband,
                self.minimum_linear_stick,
            )
            # PM01 yaw stick convention is opposite to ROS angular.z.
            yaw = shape_stick(
                -float(self.last_command.angular.z),
                self.yaw_scale,
                self.yaw_deadband,
                self.minimum_yaw_stick,
            )
            message.analog_states[2] = forward
            message.analog_states[3] = lateral
            message.analog_states[5] = yaw
            self.lcm_handle.publish(self.channel, message.encode())
        elif self.armed:
            message = GamepadKeys()
            message.timestamp = int(time.time_ns() / 1000)
            self.lcm_handle.publish(self.channel, message.encode())
        status = String()
        status.data = json.dumps(
            {
                "state": "FORWARDING" if active else "BLOCKED",
                "reason": "ok" if active else ("disarmed" if not self.armed else "command_timeout"),
                "armed": self.armed,
                "enable_forward": self.enable_forward,
                "forward_stick": round(float(forward), 4),
                "lateral_stick": round(float(lateral), 4),
                "yaw_stick": round(float(yaw), 4),
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Pm01GamepadBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
