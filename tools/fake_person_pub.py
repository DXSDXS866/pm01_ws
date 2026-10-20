#!/usr/bin/env python3
"""Fake /person/position_robot publisher for closed-loop smoke testing.

Publishes a PointStamped at 10 Hz with a fresh header stamp (required by
person_follow_controller's source-age check) in the base_link frame.

Run in an isolated domain so it never reaches the real robot:

    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
    source /opt/ros/humble/setup.bash
    source ~/pm01_ws/install/setup.bash
    python3 ~/pm01_ws/tools/fake_person_pub.py [x] [y]   # defaults: 2.0 0.0
"""

import sys

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class FakePerson(Node):
    def __init__(self, x: float, y: float) -> None:
        super().__init__("fake_person_pub")
        self.pub = self.create_publisher(PointStamped, "/person/position_robot", 10)
        self.x = x
        self.y = y
        self.create_timer(0.1, self.tick)

    def tick(self) -> None:
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.point.x = self.x
        message.point.y = self.y
        message.point.z = 0.0
        self.pub.publish(message)


def main() -> None:
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    rclpy.init()
    node = FakePerson(x, y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
