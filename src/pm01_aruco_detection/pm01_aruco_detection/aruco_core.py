"""OpenCV-only helpers shared by offline and ROS ArUco detectors."""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


SUPPORTED_DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in (
        "DICT_4X4_50",
        "DICT_4X4_100",
        "DICT_5X5_50",
        "DICT_5X5_100",
        "DICT_6X6_50",
        "DICT_6X6_100",
        "DICT_7X7_50",
        "DICT_7X7_100",
        "DICT_ARUCO_ORIGINAL",
    )
    if hasattr(cv2.aruco, name)
}


@dataclass(frozen=True)
class MarkerObservation:
    """One marker detection in image and optional camera-relative pose."""

    marker_id: int
    corners: np.ndarray
    center_px: Tuple[float, float]
    rvec: Optional[np.ndarray] = None
    tvec: Optional[np.ndarray] = None


def get_dictionary(name: str):
    """Return an OpenCV ArUco dictionary by its stable symbolic name."""
    if name not in SUPPORTED_DICTIONARIES:
        choices = ", ".join(sorted(SUPPORTED_DICTIONARIES))
        raise ValueError(f"Unsupported ArUco dictionary '{name}'. Choose one of: {choices}")
    dictionary_id = SUPPORTED_DICTIONARIES[name]
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.Dictionary_get(dictionary_id)


def create_detector_parameters():
    """Create detector parameters across OpenCV 4.5 and newer APIs."""
    # OpenCV 4.6 exposes DetectorParameters but still uses the legacy
    # detectMarkers API; passing the new constructor's object to that API can
    # segfault. Keep each detector API paired with its matching parameter API.
    if hasattr(cv2.aruco, "ArucoDetector") and hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


def generate_marker_image(dictionary_name: str, marker_id: int, pixels: int) -> np.ndarray:
    """Generate a marker image without scaling an existing bitmap."""
    if marker_id < 0:
        raise ValueError("marker_id must be non-negative")
    if pixels < 64:
        raise ValueError("pixels must be at least 64")
    dictionary = get_dictionary(dictionary_name)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)
    image = np.zeros((pixels, pixels), dtype=np.uint8)
    cv2.aruco.drawMarker(dictionary, marker_id, pixels, image, 1)
    return image


def _detect_raw(gray: np.ndarray, dictionary_name: str):
    dictionary = get_dictionary(dictionary_name)
    parameters = create_detector_parameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


def detect_markers(
    image_bgr: np.ndarray,
    dictionary_name: str,
    target_id: int = -1,
    marker_size_m: Optional[float] = None,
    camera_matrix: Optional[np.ndarray] = None,
    distortion: Optional[np.ndarray] = None,
) -> Tuple[List[MarkerObservation], Sequence[np.ndarray]]:
    """Detect markers and optionally estimate camera-relative marker poses."""
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError("input image is empty")
    if marker_size_m is not None and marker_size_m <= 0.0:
        raise ValueError("marker_size_m must be positive")

    if image_bgr.ndim == 2:
        gray = image_bgr
    else:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = _detect_raw(gray, dictionary_name)
    if ids is None:
        return [], rejected

    selected = []
    selected_ids = []
    for marker_corners, marker_id_array in zip(corners, ids):
        marker_id = int(marker_id_array[0])
        if target_id >= 0 and marker_id != target_id:
            continue
        selected.append(marker_corners)
        selected_ids.append(marker_id)

    rvecs = tvecs = None
    can_estimate_pose = (
        selected
        and marker_size_m is not None
        and camera_matrix is not None
        and distortion is not None
    )
    if can_estimate_pose:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            selected,
            marker_size_m,
            np.asarray(camera_matrix, dtype=np.float64),
            np.asarray(distortion, dtype=np.float64),
        )

    observations = []
    for index, (marker_corners, marker_id) in enumerate(zip(selected, selected_ids)):
        points = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
        center = tuple(np.mean(points, axis=0).tolist())
        rvec = None if rvecs is None else np.asarray(rvecs[index]).reshape(3)
        tvec = None if tvecs is None else np.asarray(tvecs[index]).reshape(3)
        observations.append(
            MarkerObservation(
                marker_id=marker_id,
                corners=points,
                center_px=center,
                rvec=rvec,
                tvec=tvec,
            )
        )
    return observations, rejected


def rotation_vector_to_quaternion(rvec: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert an OpenCV Rodrigues rotation vector to x/y/z/w quaternion."""
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def annotate_image(
    image_bgr: np.ndarray,
    observations: Sequence[MarkerObservation],
    camera_matrix: Optional[np.ndarray] = None,
    distortion: Optional[np.ndarray] = None,
    axis_length_m: float = 0.1,
) -> np.ndarray:
    """Draw marker borders, IDs, translations and optional pose axes."""
    result = image_bgr.copy()
    if not observations:
        cv2.putText(result, "NO TARGET", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return result

    corners = [obs.corners.astype(np.float32).reshape(1, 4, 2) for obs in observations]
    ids = np.asarray([[obs.marker_id] for obs in observations], dtype=np.int32)
    cv2.aruco.drawDetectedMarkers(result, corners, ids)
    for obs in observations:
        center = (int(round(obs.center_px[0])), int(round(obs.center_px[1])))
        label = f"id={obs.marker_id}"
        if obs.tvec is not None:
            label += f" xyz=({obs.tvec[0]:.2f},{obs.tvec[1]:.2f},{obs.tvec[2]:.2f})m"
            if camera_matrix is not None and distortion is not None:
                cv2.drawFrameAxes(
                    result,
                    np.asarray(camera_matrix, dtype=np.float64),
                    np.asarray(distortion, dtype=np.float64),
                    obs.rvec.reshape(3, 1),
                    obs.tvec.reshape(3, 1),
                    axis_length_m,
                )
        cv2.putText(
            result,
            label,
            (center[0] + 8, center[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
        )
    return result
