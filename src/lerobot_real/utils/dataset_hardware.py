"""Project-specific hardware semantics stored beside LeRobot metadata."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


HARDWARE_METADATA_RELATIVE_PATH = Path("meta") / "robot_hardware.json"
LEGACY_GRIPPER_MAX_WIDTH_M = 0.068
SIDES = ("left", "right")


def validate_hardware_metadata(metadata: dict[str, Any]) -> dict[str, float]:
    """Return validated per-side gripper widths from project metadata."""
    try:
        schema_version = int(metadata["schema_version"])
        normalized_range = metadata["gripper"]["normalized_range"]
        widths = metadata["gripper"]["max_width_m_by_side"]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("robot_hardware.json has an invalid structure") from exc
    if schema_version != 1:
        raise ValueError(f"unsupported robot_hardware.json schema_version={schema_version}")
    if normalized_range != [0.0, 1.0]:
        raise ValueError("robot_hardware.json gripper normalized_range must be [0.0, 1.0]")
    if set(widths) != set(SIDES):
        raise ValueError("robot_hardware.json must define left and right gripper widths")

    result = {side: float(widths[side]) for side in SIDES}
    if any(not math.isfinite(value) or not 0 < value <= 0.1 for value in result.values()):
        raise ValueError("robot_hardware.json gripper widths must be in (0, 0.1] metres")
    return result


def read_gripper_widths(dataset_root: Path) -> tuple[dict[str, float], bool]:
    """Read dataset gripper semantics, falling back to the legacy 68 mm schema."""
    path = dataset_root / HARDWARE_METADATA_RELATIVE_PATH
    if not path.is_file():
        return ({side: LEGACY_GRIPPER_MAX_WIDTH_M for side in SIDES}, True)
    metadata = json.loads(path.read_text(encoding="utf-8"))
    return validate_hardware_metadata(metadata), False


def ensure_hardware_metadata(
    dataset_root: Path,
    metadata: dict[str, Any] | None,
    *,
    existing_episodes: int,
) -> None:
    """Write new metadata or reject a resume that would mix physical semantics."""
    if metadata is None:
        return
    expected = validate_hardware_metadata(metadata)
    path = dataset_root / HARDWARE_METADATA_RELATIVE_PATH
    if path.is_file():
        observed = validate_hardware_metadata(
            json.loads(path.read_text(encoding="utf-8"))
        )
        if observed != expected:
            raise ValueError(
                "dataset gripper width metadata does not match the current robot config: "
                f"dataset={observed}, current={expected}"
            )
        return

    if existing_episodes > 0 and any(
        not math.isclose(value, LEGACY_GRIPPER_MAX_WIDTH_M, abs_tol=1e-9)
        for value in expected.values()
    ):
        raise ValueError(
            "existing dataset has no robot_hardware.json and is therefore a legacy "
            "68 mm dataset; refusing to resume it with a different gripper range"
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(path)
