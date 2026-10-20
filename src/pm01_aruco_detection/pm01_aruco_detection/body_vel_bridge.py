"""Fail-safe bridge from visual Twist commands to PM v2 BodyVelCmd."""

import json
import math
import time

from geometry_msgs.msg import Twist
from interface_protocol.msg import BodyVelCmd, MotionState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class BodyVelBridge(Node):
    """Bridge commands only when armed and in the configured locomotion state."""

    def __init__(self) -> None:
        super().__init__("visual_body_vel_bridge")
        self.declare_parameter("input_topic", "/vision_follow/cmd_vel")
        self.declare_parameter("output_topic", "/motion/body_vel_cmd")
        self.declare_parameter("armed", False)
        self.declare_parameter("allowed_motion_state", "rl_basic")
        self.declare_parameter("enable_forward", False)
        self.declare_parameter("max_forward_mps", 0.08)
        self.declare_parameter("max_yaw_rps", 0.50)
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("publish_rate_hz", 20.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.armed = bool(self.get_parameter("armed").value)
        self.allowed_motion_state = str(self.get_parameter("allowed_motion_state").value)
        self.enable_forward = bool(self.get_parameter("enable_forward").value)
        self.max_forward = float(self.get_parameter("max_forward_mps").value)
        self.max_yaw = float(self.get_parameter("max_yaw_rps").value)
        self.timeout = float(self.get_parameter("command_timeout_sec").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        if min(self.max_forward, self.max_yaw, self.timeout, publish_rate) <= 0.0:
            raise ValueError("limits, timeout and publish rate must be positive")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            # RELIABLE 发布端对 RELIABLE / BEST_EFFORT 订阅端都兼容;
            # BEST_EFFORT 发布端遇上 RELIABLE 订阅端会静默失配,一条都送不到。
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.command_pub = self.create_publisher(BodyVelCmd, output_topic, qos)
        self.status_pub = self.create_publisher(String, "/vision_follow/body_vel_bridge_status", 10)
        self.create_subscription(Twist, input_topic, self.on_twist, 10)
        self.create_subscription(MotionState, "/motion/motion_state", self.on_motion_state, qos)
        self.last_command = Twist()
        self.last_command_time = None
        self.motion_state = "UNKNOWN"
        self.create_timer(1.0 / publish_rate, self.on_timer)
        self.get_logger().info(
            f"BodyVel bridge ready: {input_topic} -> {output_topic}, armed={self.armed}, "
            f"allowed_state={self.allowed_motion_state}, enable_forward={self.enable_forward}, "
            f"limits={self.max_forward:.2f} m/s,{self.max_yaw:.2f} rad/s"
        )

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def on_twist(self, message: Twist) -> None:
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(float(value)) for value in values):
            self.last_command = Twist()
            self.last_command_time = None
            return
        command = Twist()
        # First real test is forward-only; reverse and lateral motion are disabled.
        command.linear.x = (
            self.clamp(float(message.linear.x), 0.0, self.max_forward)
            if self.enable_forward
            else 0.0
        )
        command.angular.z = self.clamp(float(message.angular.z), -self.max_yaw, self.max_yaw)
        self.last_command = command
        self.last_command_time = time.monotonic()

    def on_motion_state(self, message: MotionState) -> None:
        self.motion_state = str(message.current_motion_task)

    def on_timer(self) -> None:
        fresh = (
            self.last_command_time is not None
            and time.monotonic() - self.last_command_time <= self.timeout
        )
        enabled = self.armed and self.motion_state == self.allowed_motion_state and fresh
        linear = self.last_command.linear.x if enabled else 0.0
        yaw = self.last_command.angular.z if enabled else 0.0

        # When disarmed, publish no command at all. This permits passive inspection
        # without becoming a command source in the robot input arbiter.
        if self.armed:
            message = BodyVelCmd()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.linear_velocity = [float(linear), 0.0]
            message.yaw_velocity = float(yaw)
            self.command_pub.publish(message)

        state = "FORWARDING" if enabled else "BLOCKED"
        reason = "ok"
        if not self.armed:
            reason = "disarmed"
        elif self.motion_state != self.allowed_motion_state:
            reason = f"motion_state={self.motion_state}"
        elif not fresh:
            reason = "command_timeout"
        status = String()
        status.data = json.dumps(
            {
                "state": state,
                "reason": reason,
                "armed": self.armed,
                "motion_state": self.motion_state,
                "allowed_motion_state": self.allowed_motion_state,
                "enable_forward": self.enable_forward,
                "linear_mps": round(float(linear), 4),
                "yaw_rps": round(float(yaw), 4),
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BodyVelBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
