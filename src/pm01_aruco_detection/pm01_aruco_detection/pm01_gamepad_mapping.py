"""Pure-Python PM01 velocity-to-stick mapping helpers."""

import math


def clamp_stick(value: float) -> float:
    return max(-1.0, min(1.0, value))


def shape_stick(
    velocity: float,
    scale: float,
    command_deadband: float,
    minimum_stick: float,
) -> float:
    """Map a velocity command to PM01 stick space while overcoming gait deadband."""
    if scale <= 0.0:
        raise ValueError("scale must be positive")
    if command_deadband < 0.0:
        raise ValueError("command_deadband must be non-negative")
    if not 0.0 <= minimum_stick <= 1.0:
        raise ValueError("minimum_stick must be within [0, 1]")
    if abs(velocity) <= command_deadband:
        return 0.0
    scaled = clamp_stick(velocity / scale)
    return math.copysign(max(abs(scaled), minimum_stick), scaled)
