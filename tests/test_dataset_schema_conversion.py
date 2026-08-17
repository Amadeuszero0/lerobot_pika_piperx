import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "convert_dual_pika_piper_dataset_schema.py"
)
SPEC = importlib.util.spec_from_file_location("convert_dual_pika_piper_dataset_schema", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
conversion = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = conversion
SPEC.loader.exec_module(conversion)


def _legacy_names() -> tuple[list[str], list[str]]:
    state_names = [
        "left.pose.x",
        "left.pose.y",
        "left.pose.z",
        "left.pose.rx",
        "left.pose.ry",
        "left.pose.rz",
        "left.gripper.pos",
        *[f"left.joint{i}.angle_rad" for i in range(1, 7)],
        "right.pose.x",
        "right.pose.y",
        "right.pose.z",
        "right.pose.rx",
        "right.pose.ry",
        "right.pose.rz",
        "right.gripper.pos",
        *[f"right.joint{i}.angle_rad" for i in range(1, 7)],
    ]
    action_names = [
        "left.pose.x",
        "left.pose.y",
        "left.pose.z",
        "left.pose.rx",
        "left.pose.ry",
        "left.pose.rz",
        "left.gripper.pos",
        "right.pose.x",
        "right.pose.y",
        "right.pose.z",
        "right.pose.rx",
        "right.pose.ry",
        "right.pose.rz",
        "right.gripper.pos",
    ]
    return state_names, action_names


def test_convert_vectors_splits_endpose_and_builds_next_state_joint_action() -> None:
    state_names, action_names = _legacy_names()
    state_by_name = {name: float(index + 1) for index, name in enumerate(state_names)}
    next_state_by_name = {name: float(index + 101) for index, name in enumerate(state_names)}
    action_by_name = {name: float(index + 201) for index, name in enumerate(action_names)}

    converted = conversion.convert_vectors(
        [state_by_name[name] for name in state_names],
        [next_state_by_name[name] for name in state_names],
        [action_by_name[name] for name in action_names],
        state_names=state_names,
        action_names=action_names,
    )

    expected_state = np.array(
        [state_by_name[name] for name in conversion.JOINT_NAMES], dtype=np.float32
    )
    expected_endpose = np.array(
        [state_by_name[name] for name in conversion.ENDPOSE_NAMES], dtype=np.float32
    )
    expected_joint_action = np.array(
        [
            *[next_state_by_name[name] for name in conversion.JOINT_ONLY_NAMES[:6]],
            action_by_name["left.gripper.pos"],
            *[next_state_by_name[name] for name in conversion.JOINT_ONLY_NAMES[6:]],
            action_by_name["right.gripper.pos"],
        ],
        dtype=np.float32,
    )
    expected_action_endpose = np.array(
        [action_by_name[name] for name in conversion.ENDPOSE_NAMES], dtype=np.float32
    )

    np.testing.assert_array_equal(converted.observation_state, expected_state)
    np.testing.assert_array_equal(converted.observation_endpose, expected_endpose)
    np.testing.assert_array_equal(converted.action, expected_joint_action)
    np.testing.assert_array_equal(converted.action_endpose, expected_action_endpose)


def test_convert_vectors_uses_names_instead_of_assuming_legacy_order() -> None:
    state_names, action_names = _legacy_names()
    state_names = list(reversed(state_names))
    action_names = list(reversed(action_names))
    state = np.arange(len(state_names), dtype=np.float32)
    next_state = state + 100
    action = np.arange(len(action_names), dtype=np.float32) + 200

    converted = conversion.convert_vectors(
        state,
        next_state,
        action,
        state_names=state_names,
        action_names=action_names,
    )

    state_lookup = dict(zip(state_names, state, strict=True))
    assert converted.observation_state[0] == state_lookup["left.joint1.angle_rad"]
    assert converted.observation_endpose[0] == state_lookup["left.pose.x"]


def test_convert_vectors_rejects_missing_required_joint() -> None:
    state_names, action_names = _legacy_names()
    state_names.remove("right.joint6.angle_rad")

    with pytest.raises(ValueError, match="right.joint6.angle_rad"):
        conversion.convert_vectors(
            np.zeros(len(state_names), dtype=np.float32),
            np.zeros(len(state_names), dtype=np.float32),
            np.zeros(len(action_names), dtype=np.float32),
            state_names=state_names,
            action_names=action_names,
        )


def test_build_output_features_preserves_visuals_and_drops_old_codec_info() -> None:
    source_features = {
        "observation.state": {"dtype": "float32", "shape": [26], "names": []},
        "action": {"dtype": "float32", "shape": [14], "names": []},
        "observation.images.left.third_view": {
            "dtype": "video",
            "shape": [480, 640, 3],
            "names": ["height", "width", "channels"],
            "info": {"video.codec": "h264"},
        },
    }

    features = conversion.build_output_features(source_features)

    assert features["observation.state"]["shape"] == (14,)
    assert features["observation.state.endpose"]["shape"] == (12,)
    assert features["action"]["shape"] == (14,)
    assert features["action.endpose"]["shape"] == (12,)
    assert features["observation.images.left.third_view"]["shape"] == (480, 640, 3)
    assert "info" not in features["observation.images.left.third_view"]
