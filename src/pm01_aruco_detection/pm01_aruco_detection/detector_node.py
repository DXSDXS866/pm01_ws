"""ROS 2 node for ArUco ID detection and camera-relative pose estimation."""

import json
import time
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32, String

from .aruco_core import annotate_image, detect_markers, rotation_vector_to_quaternion


class ArucoDetectorNode(Node):
    """Detect a configured marker and publish a fail-safe validity signal."""

    def __init__(self) -> None:
        super().__init__("aruco_detector")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("target_id", 23)
        self.declare_parameter("marker_size_m", 0.074)
        self.declare_parameter("image_timeout_sec", 0.5)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("axis_length_m", 0.1)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.dictionary = str(self.get_parameter("dictionary").value)
        self.target_id = int(self.get_parameter("target_id").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)
        self.timeout_sec = float(self.get_parameter("image_timeout_sec").value)
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)
        self.axis_length_m = float(self.get_parameter("axis_length_m").value)
        if self.marker_size_m <= 0.0:
            raise ValueError("marker_size_m must be positive")
        if self.timeout_sec <= 0.0:
            raise ValueError("image_timeout_sec must be positive")

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.distortion: Optional[np.ndarray] = None
        self.last_image_monotonic: Optional[float] = None
        self.last_detection = False
        self.timeout_reported = False

        self.pose_pub = self.create_publisher(PoseStamped, "~/pose", 10)
        self.id_pub = self.create_publisher(Int32, "~/id", 10)
        self.detected_pub = self.create_publisher(Bool, "~/detected", 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.debug_pub = self.create_publisher(Image, "~/debug_image", qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_timer(min(0.1, self.timeout_sec / 2.0), self.on_watchdog)

        self.get_logger().info(
            f"Waiting for image={self.image_topic}, camera_info={self.camera_info_topic}, "
            f"marker={self.dictionary}:{self.target_id}, size={self.marker_size_m:.3f} m"
        )

    def on_camera_info(self, message: CameraInfo) -> None:
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            self.get_logger().error("Ignoring invalid CameraInfo with non-positive focal length")
            return
        self.camera_matrix = matrix
        self.distortion = np.asarray(message.d, dtype=np.float64)

    def _publish_detected(self, value: bool) -> None:
        self.last_detection = value
        message = Bool()
        message.data = value
        self.detected_pub.publish(message)

    def _publish_status(self, payload) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_pub.publish(message)

    def on_watchdog(self) -> None:
        if self.last_image_monotonic is None:
            return
        age = time.monotonic() - self.last_image_monotonic
        if age > self.timeout_sec and not self.timeout_reported:
            self.timeout_reported = True
            self._publish_detected(False)
            self._publish_status(
                {
                    "state": "IMAGE_TIMEOUT",
                    "detected": False,
                    "image_age_sec": round(age, 3),
                }
            )
            self.get_logger().warning(f"Image stream timeout: no frame for {age:.3f} s")

    def on_image(self, message: Image) -> None:
        self.last_image_monotonic = time.monotonic()
        self.timeout_reported = False
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            observations, _ = detect_markers(
                image,
                self.dictionary,
                self.target_id,
                self.marker_size_m,
                self.camera_matrix,
                self.distortion,
            )
        except Exception as error:
            self._publish_detected(False)
            self._publish_status({"state": "ERROR", "detected": False, "error": str(error)})
            self.get_logger().error(f"Detection failed: {error}")
            return

        detected = bool(observations)
        self._publish_detected(detected)
        status = {
            "state": "DETECTED" if detected else "NO_TARGET",
            "detected": detected,
            "target_id": self.target_id,
            "camera_calibrated": self.camera_matrix is not None,
            "frame_id": message.header.frame_id,
        }

        if detected:
            observation = observations[0]
            id_message = Int32()
            id_message.data = observation.marker_id
            self.id_pub.publish(id_message)
            status["center_px"] = [round(value, 2) for value in observation.center_px]
            if observation.tvec is not None:
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x = float(observation.tvec[0])
                pose.pose.position.y = float(observation.tvec[1])
                pose.pose.position.z = float(observation.tvec[2])
                qx, qy, qz, qw = rotation_vector_to_quaternion(observation.rvec)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                self.pose_pub.publish(pose)
                status["translation_m"] = [round(float(value), 4) for value in observation.tvec]
            else:
                status["pose_valid"] = False
                status["reason"] = "WAITING_FOR_CAMERA_INFO"

        self._publish_status(status)
        if self.publish_debug:
            debug = annotate_image(
                image,
                observations,
                self.camera_matrix,
                self.distortion,
                self.axis_length_m,
            )
            debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_message.header = message.header
            self.debug_pub.publish(debug_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetectorNode()
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
