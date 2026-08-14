#!/usr/bin/env python3
"""Validate an existing local LeRobot dataset and plan a safe resume run.

The formal recording launcher uses this before connecting any hardware.  It
prints one tab-separated line so Bash can consume the canonical dataset path,
repo id, number of fully saved episodes, and number of episodes still needed.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SAFE_DATASET_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


@dataclass(frozen=True)
class ResumePlan:
    root: Path
    repo_id: str
    saved_episodes: int
    episodes_to_record: int


def _read_info(root: Path) -> dict[str, Any]:
    info_path = root / "meta" / "info.json"
    if not info_path.is_file():
        raise ValueError(f"missing LeRobot metadata file: {info_path}")
    try:
        info = json.loads(info_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON in {info_path}: {exc}") from exc
    if not isinstance(info, dict):
        raise ValueError(f"{info_path} must contain a JSON object")
    return info


def build_resume_plan(dataset_root: str | Path, target_episodes: int) -> ResumePlan:
    if target_episodes <= 0:
        raise ValueError("target episodes must be a positive integer")

    root = Path(dataset_root).expanduser().resolve(strict=True)
    if not root.is_dir():
        raise ValueError(f"dataset root is not a directory: {root}")
    if any(character in str(root) for character in ("\t", "\n", "\r")):
        raise ValueError("dataset path may not contain tabs or newlines")
    if not SAFE_DATASET_NAME.fullmatch(root.name):
        raise ValueError(
            "dataset directory name may contain only letters, numbers, dot, "
            "underscore, and hyphen"
        )

    info = _read_info(root)
    required_keys = {
        "codebase_version",
        "robot_type",
        "fps",
        "features",
        "total_episodes",
        "total_frames",
    }
    missing = sorted(required_keys - set(info))
    if missing:
        raise ValueError(f"info.json is missing required fields: {', '.join(missing)}")

    saved_episodes = info["total_episodes"]
    total_frames = info["total_frames"]
    if isinstance(saved_episodes, bool) or not isinstance(saved_episodes, int):
        raise ValueError("info.json total_episodes must be an integer")
    if saved_episodes < 0:
        raise ValueError("info.json total_episodes may not be negative")
    if isinstance(total_frames, bool) or not isinstance(total_frames, int) or total_frames < 0:
        raise ValueError("info.json total_frames must be a non-negative integer")
    if not isinstance(info["features"], dict) or not info["features"]:
        raise ValueError("info.json features must be a non-empty mapping")

    if saved_episodes > 0:
        required_paths = (
            root / "data",
            root / "meta" / "episodes",
            root / "meta" / "tasks.parquet",
        )
        for required_path in required_paths:
            if not required_path.exists():
                raise ValueError(f"saved dataset is incomplete; missing: {required_path}")
        if total_frames == 0:
            raise ValueError("dataset has saved episodes but total_frames is zero")

    episodes_to_record = target_episodes - saved_episodes
    if episodes_to_record <= 0:
        raise ValueError(
            f"dataset already has {saved_episodes} episodes, which meets or exceeds "
            f"the target of {target_episodes}"
        )

    return ResumePlan(
        root=root,
        repo_id=f"local/{root.name}",
        saved_episodes=saved_episodes,
        episodes_to_record=episodes_to_record,
    )


def validate_lerobot_load(plan: ResumePlan) -> None:
    """Exercise the same local dataset loader used by the recorder."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(plan.repo_id, root=plan.root)
    loaded_episodes = int(dataset.num_episodes)
    if loaded_episodes != plan.saved_episodes:
        raise ValueError(
            "LeRobot metadata disagreement: "
            f"info.json reports {plan.saved_episodes} episodes but the dataset loader "
            f"reports {loaded_episodes}"
        )

    episode_metadata = dataset.meta.episodes
    metadata_count = 0 if episode_metadata is None else len(episode_metadata)
    if metadata_count != plan.saved_episodes:
        raise ValueError(
            "dataset was not fully finalized: "
            f"info.json reports {plan.saved_episodes} episodes but meta/episodes "
            f"contains {metadata_count}. Do not append to this dataset automatically."
        )

    if episode_metadata is not None:
        try:
            episode_indices = episode_metadata["episode_index"]
        except (KeyError, TypeError):
            episode_indices = [episode["episode_index"] for episode in episode_metadata]
        normalized_indices = [int(index) for index in episode_indices]
        expected_indices = list(range(plan.saved_episodes))
        if normalized_indices != expected_indices:
            raise ValueError(
                "meta/episodes indices are not contiguous from zero; refusing to append"
            )

    loaded_frames = len(dataset.hf_dataset)
    expected_frames = int(dataset.meta.total_frames)
    if loaded_frames != expected_frames:
        raise ValueError(
            "dataset frame count disagreement: "
            f"info.json reports {expected_frames} but data parquet contains {loaded_frames}"
        )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate a local LeRobot dataset before appending episodes"
    )
    parser.add_argument("dataset_root", help="Existing local dataset directory")
    parser.add_argument(
        "--target-episodes",
        type=int,
        required=True,
        help="Desired total episode count after this run",
    )
    parser.add_argument(
        "--metadata-only",
        action="store_true",
        help="Skip loading the dataset through LeRobot (intended for tests only)",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        plan = build_resume_plan(args.dataset_root, args.target_episodes)
        if not args.metadata_only:
            validate_lerobot_load(plan)
    except Exception as exc:
        print(f"Resume validation failed: {exc}", file=sys.stderr)
        return 1

    print(
        "\t".join(
            (
                str(plan.root),
                plan.repo_id,
                str(plan.saved_episodes),
                str(plan.episodes_to_record),
            )
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
