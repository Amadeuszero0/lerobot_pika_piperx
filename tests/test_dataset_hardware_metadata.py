import json
from pathlib import Path

import pytest

from lerobot_real.utils.dataset_hardware import (
    HARDWARE_METADATA_RELATIVE_PATH,
    ensure_hardware_metadata,
    read_gripper_widths,
)


def _metadata(width: float) -> dict:
    return {
        "schema_version": 1,
        "gripper": {
            "normalized_range": [0.0, 1.0],
            "max_width_m_by_side": {"left": width, "right": width},
        },
    }


def test_new_dataset_records_large_gripper_semantics(tmp_path: Path) -> None:
    ensure_hardware_metadata(tmp_path, _metadata(0.098), existing_episodes=0)

    path = tmp_path / HARDWARE_METADATA_RELATIVE_PATH
    assert json.loads(path.read_text(encoding="utf-8")) == _metadata(0.098)
    assert read_gripper_widths(tmp_path) == (
        {"left": 0.098, "right": 0.098},
        False,
    )


def test_legacy_dataset_without_metadata_defaults_to_68_mm(tmp_path: Path) -> None:
    assert read_gripper_widths(tmp_path) == (
        {"left": 0.068, "right": 0.068},
        True,
    )


def test_resume_refuses_to_mix_legacy_and_large_gripper_semantics(
    tmp_path: Path,
) -> None:
    with pytest.raises(ValueError, match="legacy 68 mm dataset"):
        ensure_hardware_metadata(tmp_path, _metadata(0.098), existing_episodes=1)


def test_resume_refuses_mismatched_recorded_width(tmp_path: Path) -> None:
    ensure_hardware_metadata(tmp_path, _metadata(0.098), existing_episodes=0)

    with pytest.raises(ValueError, match="does not match"):
        ensure_hardware_metadata(tmp_path, _metadata(0.068), existing_episodes=1)
