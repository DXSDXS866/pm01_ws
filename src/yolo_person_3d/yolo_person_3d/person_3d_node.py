#!/usr/bin/env python3

import os
import sys

# 免疫补丁:PYTHONNOUSERSITE=1(例如按 README 跑过 colcon test 的终端)会把
# ~/.local 从 sys.path 拿掉,而 pycuda / numpy 1.26.4 恰恰装在那里,节点会在
# import 阶段死于 ModuleNotFoundError: No module named 'pycuda'。这里在 numpy/
# cv2 之前强制把 user-site 加回 sys.path(插在系统 dist-packages 之前,与
# site.py 的优先级一致)。user-site 正常启用时此补丁不做任何事。
import site

_USER_SITE = os.path.expanduser(
    "~/.local/lib/python%d.%d/site-packages"
    % (sys.version_info.major, sys.version_info.minor)
)
if os.path.isdir(_USER_SITE) and _USER_SITE not in sys.path:
    _pos = next(
        (i for i, p in enumerate(sys.path)
         if "dist-packages" in p or "site-packages" in p),
        len(sys.path),
    )
    sys.path.insert(_pos, _USER_SITE)
    site.addsitedir(_USER_SITE)  # 处理该目录下的 .pth 文件;已插入不会重复
    print("[person_3d_node] user-site 被环境屏蔽,已强制回加: %s" % _USER_SITE,
          file=sys.stderr)

import json
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformException,
    TransformListener,
)


def _default_model_path() -> str:
    """Resolve the TensorRT engine from the installed share directory.

    Falls back to the source tree when the package is not installed (for
    example when running the node script directly from src/).
    """
    try:
        return os.path.join(
            get_package_share_directory("yolo_person_3d"),
            "weights", "yolo26n.engine",
        )
    except Exception:
        return os.path.normpath(os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "weights", "yolo26n.engine",
        ))


class Person3DNode(Node):
    def __init__(self):
        super().__init__("person_3d_node")
        self.declare_parameter(
            "model_path",
            _default_model_path(),
        )
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("confidence", 0.5)
        self.declare_parameter("filter_alpha", 0.2)
        self.declare_parameter("max_jump_m", 0.8)
        self.declare_parameter("show_window", True)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("camera_position_topic", "/person/position_camera")
        self.declare_parameter("robot_position_topic", "/person/position_robot")
        self.declare_parameter("target_timeout_sec", 0.75)
        self.declare_parameter("max_depth_age_sec", 0.25)
        self.declare_parameter("max_rgb_depth_skew_sec", 0.15)
        self.declare_parameter("camera_mount_x_m", 0.0)
        self.declare_parameter("camera_mount_y_m", 0.0)
        self.declare_parameter("camera_mount_z_m", 0.0)
        self.declare_parameter("camera_mount_pitch_deg", 16.1)

        model_path = self.get_parameter("model_path").value
        self.target_frame = self.get_parameter("target_frame").value
        self.confidence = float(self.get_parameter("confidence").value)
        self.filter_alpha = float(self.get_parameter("filter_alpha").value)
        self.max_jump_m = float(self.get_parameter("max_jump_m").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.device = self.get_parameter("device").value
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.camera_position_topic = str(self.get_parameter("camera_position_topic").value)
        self.robot_position_topic = str(self.get_parameter("robot_position_topic").value)
        self.target_timeout = float(self.get_parameter("target_timeout_sec").value)
        self.max_depth_age = float(self.get_parameter("max_depth_age_sec").value)
        self.max_rgb_depth_skew = float(self.get_parameter("max_rgb_depth_skew_sec").value)
        self.camera_mount_x = float(self.get_parameter("camera_mount_x_m").value)
        self.camera_mount_y = float(self.get_parameter("camera_mount_y_m").value)
        self.camera_mount_z = float(self.get_parameter("camera_mount_z_m").value)
        self.camera_mount_pitch = np.deg2rad(
            float(self.get_parameter("camera_mount_pitch_deg").value)
        )
        if not 0.0 < self.confidence <= 1.0:
            raise ValueError("confidence must be within (0, 1]")
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be within (0, 1]")
        if min(self.max_jump_m, self.target_timeout, self.max_depth_age, self.max_rgb_depth_skew) <= 0.0:
            raise ValueError("jump and timeout parameters must be positive")

        if model_path.endswith(".engine"):
            # TensorRT engine runs on the GPU via pycuda (native sm_87). The
            # desktop torch wheel on this Jetson lacks sm_87 kernels, so we
            # bypass ultralytics' torch-based TensorRT backend and drive the
            # engine directly. `device` is ignored on this path (always GPU).
            from yolo_person_3d.trt_detector import TRTPersonDetector
            self.model = TRTPersonDetector(
                model_path, conf=self.confidence, class_filter=0)
            self.use_trt = True
            self.get_logger().info(f"TensorRT engine (GPU): {model_path}")
        else:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.use_trt = False

        self.depth_image = None
        self.depth_encoding = None
        self.depth_stamp_ns = None
        self.current_color_stamp_ns = None
        self.last_depth_monotonic = None
        self.camera_info = None
        self.filtered_robot_xyz = None
        self.last_target_monotonic = None
        self.jump_rejections = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_camera_tf()

        self.create_subscription(
            Image, self.color_topic, self.image_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.depth_topic,
            self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.info_callback,
            qos_profile_sensor_data)
        self.camera_position_pub = self.create_publisher(
            PointStamped, self.camera_position_topic, qos_profile_sensor_data)
        self.robot_position_pub = self.create_publisher(
            PointStamped, self.robot_position_topic, qos_profile_sensor_data)
        self.detected_pub = self.create_publisher(Bool, "/person/detected", 10)
        self.status_pub = self.create_publisher(String, "/person/status", 10)
        self.get_logger().info(
            f"Waiting for color={self.color_topic}, depth={self.depth_topic}, "
            f"camera_info={self.camera_info_topic}"
        )

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
        cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
        cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def publish_static_camera_tf(self):
        stamp = self.get_clock().now().to_msg()
        base_to_camera = TransformStamped()
        base_to_camera.header.stamp = stamp
        base_to_camera.header.frame_id = "base_link"
        base_to_camera.child_frame_id = "camera_link"
        base_to_camera.transform.translation.x = self.camera_mount_x
        base_to_camera.transform.translation.y = self.camera_mount_y
        base_to_camera.transform.translation.z = self.camera_mount_z
        qx, qy, qz, qw = self.quaternion_from_euler(0.0, self.camera_mount_pitch, 0.0)
        base_to_camera.transform.rotation.x = qx
        base_to_camera.transform.rotation.y = qy
        base_to_camera.transform.rotation.z = qz
        base_to_camera.transform.rotation.w = qw
        # RealSense owns camera_link -> camera_*_optical_frame. Publishing that
        # transform here as well gives one child two competing TF parents.
        self.static_tf_broadcaster.sendTransform(base_to_camera)

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        try:
            if msg.encoding == "16UC1":
                data = np.frombuffer(msg.data, dtype=np.uint16)
                self.depth_image = data.reshape(msg.height, msg.step // 2)[:, :msg.width]
            elif msg.encoding == "32FC1":
                data = np.frombuffer(msg.data, dtype=np.float32)
                self.depth_image = data.reshape(msg.height, msg.step // 4)[:, :msg.width]
            else:
                self.get_logger().error(f"Unsupported depth encoding: {msg.encoding}")
                return
            self.depth_encoding = msg.encoding
            self.depth_stamp_ns = (
                int(msg.header.stamp.sec) * 1_000_000_000
                + int(msg.header.stamp.nanosec)
            )
            self.last_depth_monotonic = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"depth conversion failed: {exc}")

    def publish_status(self, detected, state, **extra):
        detected_msg = Bool()
        detected_msg.data = bool(detected)
        self.detected_pub.publish(detected_msg)
        status = String()
        status.data = json.dumps(
            {"detected": bool(detected), "state": state, **extra},
            separators=(",", ":"),
        )
        self.status_pub.publish(status)

    def mark_target_missing(self, state):
        if (
            self.last_target_monotonic is not None
            and time.monotonic() - self.last_target_monotonic > self.target_timeout
        ):
            self.filtered_robot_xyz = None
            self.last_target_monotonic = None
            self.jump_rejections = 0
        self.publish_status(False, state)

    def image_callback(self, msg):
        self.current_color_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        try:
            if msg.encoding in ("bgr8", "rgb8"):
                data = np.frombuffer(msg.data, dtype=np.uint8)
                frame = data.reshape(msg.height, msg.step)[:, :msg.width * 3]
                frame = frame.reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "mono8":
                data = np.frombuffer(msg.data, dtype=np.uint8)
                frame = data.reshape(msg.height, msg.step)[:, :msg.width]
            else:
                self.get_logger().error(f"Unsupported image encoding: {msg.encoding}")
                return
        except Exception as exc:
            self.get_logger().error(f"image conversion failed: {exc}")
            return

        if self.use_trt:
            boxes = self.model.detect(frame)  # list of (x1, y1, x2, y2, conf)
            annotated = frame.copy()
        else:
            result = self.model.predict(
                frame, classes=[0], conf=self.confidence,
                device=self.device, verbose=False
            )[0]
            annotated = result.plot()
            boxes = []
            for b in result.boxes:
                xyxy = b.xyxy[0].cpu().numpy()
                boxes.append((float(xyxy[0]), float(xyxy[1]),
                              float(xyxy[2]), float(xyxy[3]),
                              float(b.conf[0].cpu())))

        # Publish exactly one target per frame. Highest confidence avoids sending
        # conflicting commands when multiple people are visible.
        selected = max(boxes, key=lambda item: item[4]) if boxes else None
        if selected is None:
            self.mark_target_missing("NO_PERSON")
        else:
            x1, y1, x2, y2, conf = selected
            if self.use_trt:
                cv2.rectangle(annotated, (int(x1), int(y1)),
                              (int(x2), int(y2)), (0, 255, 0), 2)
            u = int(round((x1 + x2) / 2.0))
            v = int(round((y1 + y2) / 2.0))
            camera_xyz = self.pixel_to_3d(u, v)
            robot_xyz = None

            if camera_xyz is not None and self.camera_info is not None:
                camera_point = PointStamped()
                camera_point.header = msg.header
                camera_point.header.frame_id = self.camera_info.header.frame_id
                camera_point.point.x, camera_point.point.y, camera_point.point.z = camera_xyz
                self.camera_position_pub.publish(camera_point)
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame, camera_point.header.frame_id, Time())
                    robot_point = do_transform_point(camera_point, transform)
                    # do_transform_point copies the TRANSFORM's stamp, which is 0
                    # for the static base_link->camera_link TF. Restore the image
                    # stamp so downstream freshness checks do not reject targets.
                    robot_point.header.stamp = camera_point.header.stamp
                    raw = np.array([
                        robot_point.point.x, robot_point.point.y, robot_point.point.z
                    ], dtype=np.float32)
                    if self.filtered_robot_xyz is None:
                        self.filtered_robot_xyz = raw
                        self.jump_rejections = 0
                    elif np.linalg.norm(raw - self.filtered_robot_xyz) <= self.max_jump_m:
                        self.filtered_robot_xyz = (
                            self.filter_alpha * raw
                            + (1.0 - self.filter_alpha) * self.filtered_robot_xyz
                        )
                        self.jump_rejections = 0
                    else:
                        self.jump_rejections += 1
                        if self.jump_rejections >= 3:
                            self.filtered_robot_xyz = raw
                            self.jump_rejections = 0
                        else:
                            self.publish_status(
                                False,
                                "TARGET_JUMP_REJECTED",
                                jump_m=round(float(np.linalg.norm(raw - self.filtered_robot_xyz)), 3),
                            )
                            raw = None
                    if raw is not None:
                        robot_xyz = tuple(float(x) for x in self.filtered_robot_xyz)
                        robot_point.point.x, robot_point.point.y, robot_point.point.z = robot_xyz
                        self.robot_position_pub.publish(robot_point)
                        self.last_target_monotonic = time.monotonic()
                        self.publish_status(
                            True,
                            "TRACKING",
                            confidence=round(float(conf), 3),
                            frame_id=robot_point.header.frame_id,
                            position_m=[round(value, 4) for value in robot_xyz],
                        )
                except TransformException as exc:
                    self.get_logger().warn(f"TF unavailable: {exc}", throttle_duration_sec=2.0)
                    self.mark_target_missing("TF_UNAVAILABLE")

            if camera_xyz is None:
                self.mark_target_missing("DEPTH_UNAVAILABLE")

            if camera_xyz is not None:
                cv2.drawMarker(annotated, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                if robot_xyz is not None:
                    label = (f"base_link X={robot_xyz[0]:.2f} "
                             f"Y={robot_xyz[1]:.2f} Z={robot_xyz[2]:.2f}m")
                else:
                    label = "base_link XYZ unavailable"
                cv2.putText(annotated, label, (int(x1), max(20, int(y1) - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            self.get_logger().info(
                f"person conf={conf:.2f}, pixel=({u},{v}), "
                f"camera_xyz={camera_xyz}, robot_xyz={robot_xyz}")

        if self.show_window:
            cv2.imshow("YOLO person 3D", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rclpy.shutdown()

    def pixel_to_3d(self, u, v):
        if self.depth_image is None or self.camera_info is None:
            return None
        if (
            self.last_depth_monotonic is None
            or time.monotonic() - self.last_depth_monotonic > self.max_depth_age
        ):
            return None
        if self.depth_stamp_ns is not None and self.current_color_stamp_ns is not None:
            skew = abs(self.current_color_stamp_ns - self.depth_stamp_ns) / 1e9
            if skew > self.max_rgb_depth_skew:
                return None
        height, width = self.depth_image.shape[:2]
        if not (0 <= u < width and 0 <= v < height):
            return None
        radius = 5
        x0, x1 = max(0, u - radius), min(width, u + radius + 1)
        y0, y1 = max(0, v - radius), min(height, v + radius + 1)
        values = self.depth_image[y0:y1, x0:x1].astype(np.float32)
        values = values[np.isfinite(values) & (values > 0)]
        if values.size == 0:
            return None
        z = float(np.median(values))
        if self.depth_encoding == "16UC1":
            z *= 0.001
        if not (0.1 < z < 10.0):
            return None
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        if fx <= 0 or fy <= 0:
            return None
        return float((u - cx) * z / fx), float((v - cy) * z / fy), z


def main(args=None):
    rclpy.init(args=args)
    node = Person3DNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
