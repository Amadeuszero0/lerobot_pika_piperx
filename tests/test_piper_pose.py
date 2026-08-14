import math

import numpy as np
import pytest

from lerobot_real.devices.piper.pose import (
    axis_angle_to_rpy_degrees,
    clamp,
    rotation_distance,
    rotation_step_towards,
    rpy_degrees_to_axis_angle,
    vector_step_towards,
)
from lerobot_real.devices.umi.vive_tracker.transformations import Transformations


@pytest.mark.parametrize(
    ("value", "bounds", "expected"),
    [
        (3.0, None, 3.0),
        (-2.0, (0.0, 10.0), 0.0),
        (4.0, (0.0, 10.0), 4.0),
        (12.0, (0.0, 10.0), 10.0),
    ],
)
def test_clamp(value: float, bounds: tuple[float, float] | None, expected: float) -> None:
    assert clamp(value, bounds) == expected


def test_near_identity_rotation_matrix_returns_finite_zero_axis_angle() -> None:
    rotation = np.eye(3)
    rotation[0, 0] -= 1e-14

    result = Transformations.rotation_matrix_to_rxryrz(rotation)

    assert np.all(np.isfinite(result))
    assert result == pytest.approx(np.zeros(3))


def test_vector_step_towards_limits_euclidean_distance() -> None:
    result = vector_step_towards((0.0, 0.0, 0.0), (3.0, 4.0, 0.0), 2.0)
    assert result == pytest.approx((1.2, 1.6, 0.0))


def test_vector_step_towards_returns_nearby_target_unchanged() -> None:
    target = (1.0, 2.0, 3.0)
    assert vector_step_towards((0.0, 0.0, 0.0), target, 10.0) == target


def test_rotation_step_uses_short_path_across_pi_boundary() -> None:
    current = (0.0, 0.0, math.pi - 0.01)
    target = (0.0, 0.0, -math.pi + 0.01)

    limited = rotation_step_towards(current, target, max_step=0.005)

    current_matrix = Transformations.rxryrz_to_rotation_matrix(*current)
    limited_matrix = Transformations.rxryrz_to_rotation_matrix(*limited)
    target_matrix = Transformations.rxryrz_to_rotation_matrix(*target)
    step = Transformations.rotation_matrix_to_rxryrz(current_matrix.T @ limited_matrix)
    remaining = Transformations.rotation_matrix_to_rxryrz(limited_matrix.T @ target_matrix)
    assert np.linalg.norm(step) == pytest.approx(0.005)
    assert np.linalg.norm(remaining) == pytest.approx(0.015)


def test_rotation_distance_uses_short_path_across_pi_boundary() -> None:
    current = (0.0, 0.0, math.pi - 0.01)
    target = (0.0, 0.0, -math.pi + 0.01)

    assert rotation_distance(current, target) == pytest.approx(0.02)


@pytest.mark.parametrize(
    "axis_angle",
    [
        (0.0, 0.0, 0.0),
        (0.1, -0.2, 0.3),
        (-0.3, 0.15, 0.05),
    ],
)
def test_axis_angle_rpy_round_trip(axis_angle: tuple[float, float, float]) -> None:
    rpy_degrees = axis_angle_to_rpy_degrees(*axis_angle)
    recovered = rpy_degrees_to_axis_angle(*rpy_degrees)
    assert recovered == pytest.approx(axis_angle, abs=1e-7)
