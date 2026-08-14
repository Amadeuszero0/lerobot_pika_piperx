import numpy as np
import pytest

from lerobot_real.deployment.action_utils import (
    PolicyObservationBuilder,
    extract_action_chunk,
    make_robot_action,
    process_action_chunk,
)

PIPER_POSE_KEYS = (
    "pose.x",
    "pose.y",
    "pose.z",
    "pose.rx",
    "pose.ry",
    "pose.rz",
    "gripper.pos",
)


def test_observation_builder_matches_reference_request_layout() -> None:
    observation = {key: float(index) for index, key in enumerate(PIPER_POSE_KEYS)}
    observation["image"] = np.ones((3, 4, 3), dtype=np.float32)

    request = PolicyObservationBuilder(
        camera_names=("image",),
        state_keys=PIPER_POSE_KEYS,
    ).build(observation, "pick the block")

    assert set(request) == {
        "video.image",
        "state",
        "annotation.human.task_description",
    }
    assert request["state"].shape == (1, 7)
    assert request["state"].dtype == np.float32
    assert request["video.image"].shape == (1, 3, 4, 3)
    assert request["video.image"].dtype == np.uint8
    assert request["annotation.human.task_description"] == ["pick the block"]


def test_extract_cartesian_component_action_chunk() -> None:
    action = {
        "action.position": np.zeros((3, 3)),
        "action.rotation": np.ones((3, 3)),
        "action.gripper": np.array([[0.1], [0.2], [0.3]]),
    }

    chunk = extract_action_chunk(action, action_space="cartesian")

    assert chunk.shape == (3, 7)
    np.testing.assert_array_equal(chunk[:, :3], 0.0)
    np.testing.assert_array_equal(chunk[:, 3:6], 1.0)
    np.testing.assert_allclose(chunk[:, 6], [0.1, 0.2, 0.3])


def test_extract_named_piper_action_chunk() -> None:
    action = {key: np.array([index, index + 1]) for index, key in enumerate(PIPER_POSE_KEYS)}

    chunk = extract_action_chunk(
        action,
        action_space="cartesian",
        action_keys=PIPER_POSE_KEYS,
    )

    assert chunk.shape == (2, 7)
    np.testing.assert_array_equal(chunk[0], np.arange(7))
    np.testing.assert_array_equal(chunk[1], np.arange(1, 8))


def test_delta_chunk_accumulates_arm_but_not_gripper() -> None:
    state = np.array([100.0, 20.0, 300.0, 0.0, 0.0, 0.0, 0.5])
    delta = np.array(
        [
            [1.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.2],
            [2.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.8],
        ]
    )

    result = process_action_chunk(state, delta, action_type="delta_endpose")

    np.testing.assert_allclose(result[:, 0], [101.0, 103.0])
    np.testing.assert_allclose(result[:, 3], [0.01, 0.02])
    np.testing.assert_allclose(result[:, 6], [0.2, 0.8])


def test_robot_action_rejects_nonfinite_and_clips_gripper() -> None:
    action = make_robot_action(
        np.array([1, 2, 3, 4, 5, 6, 2.0]),
        PIPER_POSE_KEYS,
        gripper_bounds=(0.0, 1.0),
    )
    assert action["gripper.pos"] == 1.0

    with pytest.raises(ValueError, match="finite"):
        make_robot_action(
            np.array([1, 2, 3, 4, 5, np.nan, 0.5]),
            PIPER_POSE_KEYS,
        )
