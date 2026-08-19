import json
import math
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import pytest

from lerobot_real.robots.piper.piper_follower import DUAL_JOINT_STATE_NAMES
from lerobot_real.scripts.replay_dual_piper_dataset import (
    EpisodeReplay,
    _split_action,
    load_episode,
    validate_episode,
)


def _episode(actions: np.ndarray) -> EpisodeReplay:
    count = len(actions)
    return EpisodeReplay(
        root=Path("/tmp/test"),
        episode_index=0,
        fps=30.0,
        frame_indices=np.arange(count),
        timestamps_s=np.arange(count) / 30.0,
        actions=np.asarray(actions, dtype=np.float64),
        gripper_max_width_m_by_side={"left": 0.098, "right": 0.098},
        legacy_gripper_metadata=False,
    )


def _safe_actions(count: int = 3) -> np.ndarray:
    action = np.zeros((count, 14), dtype=np.float64)
    action[:, 1] = 0.5
    action[:, 2] = -0.5
    action[:, 7 + 1] = 0.6
    action[:, 7 + 2] = -0.6
    action[:, 6] = 0.5
    action[:, 13] = 0.4
    return action


def test_validate_episode_accepts_safe_joint_actions() -> None:
    report = validate_episode(
        _episode(_safe_actions()),
        max_frame_joint_step_rad=math.radians(5),
        max_frame_gripper_step=0.25,
    )
    assert report.max_joint_step_rad == 0
    assert report.max_gripper_step == 0


def test_validate_episode_rejects_joint_limit_violation() -> None:
    actions = _safe_actions()
    actions[1, 4] = math.radians(71)
    with pytest.raises(ValueError, match="left.joint5"):
        validate_episode(
            _episode(actions),
            max_frame_joint_step_rad=math.radians(90),
            max_frame_gripper_step=1,
        )


def test_validate_episode_rejects_frame_jump() -> None:
    actions = _safe_actions()
    actions[1:, 0] = math.radians(6)
    with pytest.raises(ValueError, match="jumps 6.000 deg"):
        validate_episode(
            _episode(actions),
            max_frame_joint_step_rad=math.radians(5),
            max_frame_gripper_step=0.25,
        )


def test_split_action_converts_normalized_gripper_to_metres() -> None:
    targets = _split_action(
        _safe_actions(1)[0], {"left": 0.098, "right": 0.098}
    )
    assert targets["left"][6] == pytest.approx(0.049)
    assert targets["right"][6] == pytest.approx(0.0392)


def test_load_episode_filters_sorts_and_reorders(tmp_path: Path) -> None:
    root = tmp_path / "dataset"
    (root / "meta").mkdir(parents=True)
    (root / "data" / "chunk-000").mkdir(parents=True)
    reversed_names = list(reversed(DUAL_JOINT_STATE_NAMES))
    info = {
        "fps": 30,
        "features": {
            "action": {"dtype": "float32", "shape": [14], "names": reversed_names}
        },
    }
    (root / "meta" / "info.json").write_text(
        json.dumps(info), encoding="utf-8"
    )

    canonical = _safe_actions(2)
    recorded = canonical[:, ::-1]
    table = pa.table(
        {
            "action": pa.array([recorded[1].tolist(), recorded[0].tolist()]),
            "episode_index": pa.array([2, 2], type=pa.int64()),
            "frame_index": pa.array([1, 0], type=pa.int64()),
            "timestamp": pa.array([1 / 30, 0.0], type=pa.float32()),
        }
    )
    pq.write_table(table, root / "data" / "chunk-000" / "file-000.parquet")

    loaded = load_episode(root, 2)
    assert loaded.episode_index == 2
    np.testing.assert_allclose(loaded.actions, canonical)
    np.testing.assert_array_equal(loaded.frame_indices, [0, 1])
    assert loaded.gripper_max_width_m_by_side == {"left": 0.068, "right": 0.068}
    assert loaded.legacy_gripper_metadata is True
