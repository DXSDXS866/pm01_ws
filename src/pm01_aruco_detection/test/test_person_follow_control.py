import math

import pytest

from pm01_aruco_detection.person_follow_control import (
    compute_follow_command,
    validate_limits,
)


DEFAULTS = {
    "desired_distance_m": 1.2,
    "min_distance_m": 0.5,
    "max_linear_mps": 0.08,
    "max_angular_rps": 0.30,
    "linear_gain": 0.50,
    "angular_gain": 1.20,
    "distance_deadband_m": 0.10,
    "lateral_deadband_m": 0.05,
    "turn_in_place_angle_rad": 0.60,
}


def command(forward, lateral):
    return compute_follow_command(forward, lateral, **DEFAULTS)


def test_centered_distant_target_moves_forward_with_limit():
    result = command(2.0, 0.0)
    assert result.state == "TRACKING"
    assert result.linear_x == pytest.approx(0.08)
    assert result.angular_z == pytest.approx(0.0)


def test_target_on_left_commands_positive_ros_yaw():
    result = command(1.2, 0.3)
    assert result.angular_z > 0.0


def test_too_close_and_non_finite_targets_stop():
    assert command(0.4, 0.0).state == "STOP_TOO_CLOSE"
    assert command(math.nan, 0.0).state == "STOP_INVALID_TARGET"


def test_large_heading_turns_in_place():
    result = command(1.5, 1.5)
    assert result.linear_x == 0.0
    assert result.angular_z == pytest.approx(0.30)


def test_invalid_limits_are_rejected():
    invalid = dict(DEFAULTS)
    invalid["max_angular_rps"] = -0.3
    with pytest.raises(ValueError, match="velocity limits"):
        validate_limits(**invalid)
