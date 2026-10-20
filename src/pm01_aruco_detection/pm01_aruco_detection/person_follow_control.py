"""Pure control helpers for following a target expressed in ``base_link``."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FollowCommand:
    """One safety-limited planar velocity command."""

    linear_x: float
    angular_z: float
    state: str
    reason: str
    distance_error: float
    heading_error: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def validate_limits(
    desired_distance_m: float,
    min_distance_m: float,
    max_linear_mps: float,
    max_angular_rps: float,
    linear_gain: float,
    angular_gain: float,
    distance_deadband_m: float,
    lateral_deadband_m: float,
    turn_in_place_angle_rad: float,
) -> None:
    values = (
        desired_distance_m,
        min_distance_m,
        max_linear_mps,
        max_angular_rps,
        linear_gain,
        angular_gain,
        distance_deadband_m,
        lateral_deadband_m,
        turn_in_place_angle_rad,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("follow parameters must be finite")
    if min_distance_m <= 0.0:
        raise ValueError("min_distance_m must be positive")
    if desired_distance_m <= min_distance_m:
        raise ValueError("desired_distance_m must be greater than min_distance_m")
    if max_linear_mps <= 0.0 or max_angular_rps <= 0.0:
        raise ValueError("velocity limits must be positive")
    if linear_gain <= 0.0 or angular_gain <= 0.0:
        raise ValueError("controller gains must be positive")
    if distance_deadband_m < 0.0 or lateral_deadband_m < 0.0:
        raise ValueError("deadbands must be non-negative")
    if not 0.0 < turn_in_place_angle_rad <= math.pi / 2.0:
        raise ValueError("turn_in_place_angle_rad must be within (0, pi/2]")


def compute_follow_command(
    forward_m: float,
    lateral_m: float,
    *,
    desired_distance_m: float,
    min_distance_m: float,
    max_linear_mps: float,
    max_angular_rps: float,
    linear_gain: float,
    angular_gain: float,
    distance_deadband_m: float,
    lateral_deadband_m: float,
    turn_in_place_angle_rad: float,
) -> FollowCommand:
    """Convert a target position in ``base_link`` into a safe Twist subset."""
    if not (math.isfinite(forward_m) and math.isfinite(lateral_m)):
        return FollowCommand(0.0, 0.0, "STOP_INVALID_TARGET", "non_finite", 0.0, 0.0)
    if forward_m <= 0.0:
        return FollowCommand(0.0, 0.0, "STOP_INVALID_TARGET", "target_not_in_front", 0.0, 0.0)

    distance_error = forward_m - desired_distance_m
    heading_error = math.atan2(lateral_m, forward_m)
    if forward_m < min_distance_m:
        return FollowCommand(
            0.0,
            0.0,
            "STOP_TOO_CLOSE",
            "inside_min_distance",
            distance_error,
            heading_error,
        )

    effective_lateral = 0.0 if abs(lateral_m) <= lateral_deadband_m else heading_error
    angular = clamp(
        angular_gain * effective_lateral,
        -max_angular_rps,
        max_angular_rps,
    )
    linear = 0.0
    if abs(distance_error) > distance_deadband_m and distance_error > 0.0:
        linear = clamp(linear_gain * distance_error, 0.0, max_linear_mps)
    if abs(heading_error) >= turn_in_place_angle_rad:
        linear = 0.0

    return FollowCommand(
        linear,
        angular,
        "TRACKING",
        "ok",
        distance_error,
        heading_error,
    )
