"""Pose conversion and bounded-step helpers for Piper control.

Adapted from AgRoboticsResearch/lerobot_robot_piper under Apache-2.0.
"""

import math

from lerobot_real.devices.umi.vive_tracker.transformations import Transformations


def axis_angle_to_rpy_degrees(rx: float, ry: float, rz: float) -> tuple[float, float, float]:
    rotation = Transformations.rxryrz_to_rotation_matrix(rx, ry, rz)
    roll, pitch, yaw = Transformations.rotation_matrix_to_rpy(rotation)
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def rpy_degrees_to_axis_angle(
    roll_deg: float, pitch_deg: float, yaw_deg: float
) -> tuple[float, float, float]:
    rotation = Transformations.rpy_to_rotation_matrix(
        math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    )
    result = Transformations.rotation_matrix_to_rxryrz(rotation)
    return tuple(float(value) for value in result)


def rotation_distance(
    current: tuple[float, float, float],
    target: tuple[float, float, float],
) -> float:
    """Return the shortest angular distance between two axis-angle rotations."""
    current_matrix = Transformations.rxryrz_to_rotation_matrix(*current)
    target_matrix = Transformations.rxryrz_to_rotation_matrix(*target)
    relative_matrix = current_matrix.T @ target_matrix
    cos_angle = (float(relative_matrix.trace()) - 1.0) / 2.0
    return math.acos(min(1.0, max(-1.0, cos_angle)))


def clamp(value: float, bounds: tuple[float, float] | None) -> float:
    if bounds is None:
        return value
    return min(bounds[1], max(bounds[0], value))


def vector_step_towards(
    current: tuple[float, ...], target: tuple[float, ...], max_step: float
) -> tuple[float, ...]:
    delta = tuple(
        target_value - current_value
        for current_value, target_value in zip(current, target, strict=True)
    )
    norm = math.sqrt(sum(value * value for value in delta))
    if norm <= max_step or norm == 0.0:
        return target
    scale = max_step / norm
    return tuple(
        current_value + value * scale for current_value, value in zip(current, delta, strict=True)
    )


def rotation_step_towards(
    current: tuple[float, float, float],
    target: tuple[float, float, float],
    max_step: float,
) -> tuple[float, float, float]:
    """Step between axis-angle rotations along the shortest relative rotation."""
    current_matrix = Transformations.rxryrz_to_rotation_matrix(*current)
    target_matrix = Transformations.rxryrz_to_rotation_matrix(*target)
    relative = Transformations.rotation_matrix_to_rxryrz(current_matrix.T @ target_matrix)
    angle = math.sqrt(sum(float(value) ** 2 for value in relative))
    if angle <= max_step or angle == 0.0:
        canonical_target = Transformations.rotation_matrix_to_rxryrz(target_matrix)
        return tuple(float(value) for value in canonical_target)

    scale = max_step / angle
    step_matrix = Transformations.rxryrz_to_rotation_matrix(
        *(float(value) * scale for value in relative)
    )
    limited_matrix = current_matrix @ step_matrix
    limited = Transformations.rotation_matrix_to_rxryrz(limited_matrix)
    return tuple(float(value) for value in limited)
