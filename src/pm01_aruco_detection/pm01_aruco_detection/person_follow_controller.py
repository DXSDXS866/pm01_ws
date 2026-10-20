"""Follow ``/person/position_robot`` while publishing only an isolated command."""

import json
import time

from geometry_msgs.msg import PointStamped, Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String

from .person_follow_control import compute_follow_command, validate_limits


class PersonFollowController(Node):
    """Convert a fresh person position in ``base_link`` into a limited Twist."""

    def __init__(self) -> None:
        super().__init__("person_follow_controller")
        self.declare_parameter("target_topic", "/person/position_robot")
        self.declare_parameter("cmd_topic", "/vision_follow/cmd_vel")
        self.declare_parameter("status_topic", "/vision_follow/status")
        self.declare_parameter("required_frame", "base_link")
        self.declare_parameter("desired_distance_m", 1.2)
        self.declare_parameter("min_distance_m", 0.5)
        self.declare_parameter("max_linear_mps", 0.08)
        self.declare_parameter("max_angular_rps", 0.30)
        self.declare_parameter("linear_gain", 0.50)
        self.declare_parameter("angular_gain", 1.20)
        self.declare_parameter("distance_deadband_m", 0.10)
        self.declare_parameter("lateral_deadband_m", 0.05)
        self.declare_parameter("turn_in_place_angle_rad", 0.60)
        self.declare_parameter("target_timeout_sec", 0.50)
        self.declare_parameter("max_source_age_sec", 0.50)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.required_frame = str(self.get_parameter("required_frame").value)
        self.desired = float(self.get_parameter("desired_distance_m").value)
        self.min_distance = float(self.get_parameter("min_distance_m").value)
        self.max_linear = float(self.get_parameter("max_linear_mps").value)
        self.max_angular = float(self.get_parameter("max_angular_rps").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.distance_deadband = float(self.get_parameter("distance_deadband_m").value)
        self.lateral_deadband = float(self.get_parameter("lateral_deadband_m").value)
        self.turn_in_place_angle = float(self.get_parameter("turn_in_place_angle_rad").value)
        self.target_timeout = float(self.get_parameter("target_timeout_sec").value)
        self.max_source_age = float(self.get_parameter("max_source_age_sec").value)
        validate_limits(
            self.desired,
            self.min_distance,
            self.max_linear,
            self.max_angular,
            self.linear_gain,
            self.angular_gain,
            self.distance_deadband,
            self.lateral_deadband,
            self.turn_in_place_angle,
        )
        if self.target_timeout <= 0.0 or self.max_source_age <= 0.0:
            raise ValueError("target timeouts must be positive")

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(
            PointStamped,
            self.target_topic,
            self.on_target,
            qos_profile_sensor_data,
        )
        self.last_target_monotonic = None
        self.last_state = "WAITING_FOR_TARGET"
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            f"Person follow: target={self.target_topic}, cmd={self.cmd_topic}, "
            f"frame={self.required_frame}, desired={self.desired:.2f} m"
        )

    def publish(self, linear: float, angular: float, state: str, reason: str, **extra) -> None:
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.cmd_pub.publish(command)
        payload = {
            "state": state,
            "reason": reason,
            "linear_mps": round(float(linear), 4),
            "angular_rps": round(float(angular), 4),
            **extra,
        }
        status = String()
        status.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(status)
        if state != self.last_state:
            # Log everything as WARN to avoid mixing INFO/DEBUG levels which rclpy rejects.
            self.get_logger().warning(f"{state}: {reason}")
            self.last_state = state

    def publish_stop(self, state: str, reason: str, **extra) -> None:
        self.publish(0.0, 0.0, state, reason, **extra)

    def on_target(self, message: PointStamped) -> None:
        if message.header.frame_id != self.required_frame:
            self.last_target_monotonic = None
            self.publish_stop(
                "STOP_FRAME_MISMATCH",
                f"expected={self.required_frame},received={message.header.frame_id}",
            )
            return

        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds <= 0:
            self.last_target_monotonic = None
            self.publish_stop("STOP_INVALID_TIMESTAMP", "zero_source_stamp")
            return
        source_age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if source_age < -0.1 or source_age > self.max_source_age:
            self.last_target_monotonic = None
            self.publish_stop(
                "STOP_STALE_TARGET",
                "source_timestamp_out_of_range",
                source_age_sec=round(source_age, 3),
            )
            return

        forward = float(message.point.x)
        lateral = float(message.point.y)
        result = compute_follow_command(
            forward,
            lateral,
            desired_distance_m=self.desired,
            min_distance_m=self.min_distance,
            max_linear_mps=self.max_linear,
            max_angular_rps=self.max_angular,
            linear_gain=self.linear_gain,
            angular_gain=self.angular_gain,
            distance_deadband_m=self.distance_deadband,
            lateral_deadband_m=self.lateral_deadband,
            turn_in_place_angle_rad=self.turn_in_place_angle,
        )
        if result.state == "TRACKING":
            self.last_target_monotonic = time.monotonic()
        else:
            self.last_target_monotonic = None
        self.publish(
            result.linear_x,
            result.angular_z,
            result.state,
            result.reason,
            forward_m=round(forward, 4),
            lateral_m=round(lateral, 4),
            heading_error_rad=round(result.heading_error, 4),
            source_age_sec=round(source_age, 3),
        )

    def on_timer(self) -> None:
        if self.last_target_monotonic is None:
            if self.last_state == "WAITING_FOR_TARGET":
                self.publish_stop("WAITING_FOR_TARGET", "no_valid_target")
            return
        if time.monotonic() - self.last_target_monotonic > self.target_timeout:
            self.last_target_monotonic = None
            self.publish_stop("STOP_TARGET_TIMEOUT", "target_timeout")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonFollowController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop("STOP_SHUTDOWN", "node_shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
