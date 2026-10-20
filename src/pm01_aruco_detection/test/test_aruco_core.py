import cv2
import numpy as np
import pytest

from pm01_aruco_detection.aruco_core import (
    detect_markers,
    generate_marker_image,
    get_dictionary,
    rotation_vector_to_quaternion,
)


def test_unknown_dictionary_is_rejected():
    with pytest.raises(ValueError, match="Unsupported ArUco dictionary"):
        get_dictionary("NOT_A_DICTIONARY")


def test_marker_generation_rejects_invalid_values():
    with pytest.raises(ValueError, match="non-negative"):
        generate_marker_image("DICT_4X4_50", -1, 500)
    with pytest.raises(ValueError, match="at least 64"):
        generate_marker_image("DICT_4X4_50", 23, 32)


def test_generated_marker_is_detected_with_correct_id():
    marker = generate_marker_image("DICT_4X4_50", 23, 500)
    canvas = np.full((700, 700), 255, dtype=np.uint8)
    canvas[100:600, 100:600] = marker
    image = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    observations, _ = detect_markers(image, "DICT_4X4_50", target_id=23)
    assert len(observations) == 1
    assert observations[0].marker_id == 23
    assert observations[0].center_px == pytest.approx((349.5, 349.5), abs=1.0)


def test_target_filter_rejects_other_marker_id():
    marker = generate_marker_image("DICT_4X4_50", 23, 500)
    canvas = np.full((700, 700), 255, dtype=np.uint8)
    canvas[100:600, 100:600] = marker
    observations, _ = detect_markers(canvas, "DICT_4X4_50", target_id=7)
    assert observations == []


def test_pose_is_estimated_with_calibration():
    marker = generate_marker_image("DICT_4X4_50", 23, 400)
    canvas = np.full((600, 800), 255, dtype=np.uint8)
    canvas[100:500, 200:600] = marker
    camera_matrix = np.asarray([[600.0, 0.0, 400.0], [0.0, 600.0, 300.0], [0.0, 0.0, 1.0]])
    observations, _ = detect_markers(
        canvas,
        "DICT_4X4_50",
        target_id=23,
        marker_size_m=0.2,
        camera_matrix=camera_matrix,
        distortion=np.zeros(5),
    )
    assert len(observations) == 1
    assert observations[0].tvec is not None
    assert observations[0].tvec[2] == pytest.approx(0.3, rel=0.03)


def test_zero_rotation_converts_to_identity_quaternion():
    quaternion = rotation_vector_to_quaternion(np.zeros(3))
    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-8)
