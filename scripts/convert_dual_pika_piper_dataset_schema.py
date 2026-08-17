#!/usr/bin/env python3
"""Convert a legacy dual-Pika/Piper dataset to the split joint/EEF schema.

The legacy dataset stores joint feedback and end-effector feedback together in
``observation.state`` and stores only Cartesian commands in ``action``.  The
converted dataset contains:

* ``observation.state``: joint feedback plus gripper feedback (14 values)
* ``observation.state.endpose``: measured Cartesian poses (12 values)
* ``action``: next-frame joint feedback proxy plus commanded grippers (14 values)
* ``action.endpose``: commanded Cartesian poses (12 values)

The source dataset is never modified.  Since the exact historical IK joint
commands were not recorded, ``action`` uses the explicitly requested fallback
``joint_state[t + 1]``.  The terminal frame of every episode is omitted because
it has no valid next-frame joint target.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np


JOINT_NAMES = tuple(
    [f"left.joint{i}.angle_rad" for i in range(1, 7)]
    + ["left.gripper.pos"]
    + [f"right.joint{i}.angle_rad" for i in range(1, 7)]
    + ["right.gripper.pos"]
)
ENDPOSE_NAMES = tuple(
    [f"left.pose.{axis}" for axis in ("x", "y", "z", "rx", "ry", "rz")]
    + [f"right.pose.{axis}" for axis in ("x", "y", "z", "rx", "ry", "rz")]
)
JOINT_ONLY_NAMES = tuple(name for name in JOINT_NAMES if ".joint" in name)
GRIPPER_NAMES = ("left.gripper.pos", "right.gripper.pos")


@dataclass(frozen=True)
class ConvertedVectors:
    observation_state: np.ndarray
    observation_endpose: np.ndarray
    action: np.ndarray
    action_endpose: np.ndarray


@dataclass(frozen=True)
class ConversionSummary:
    source_root: Path
    output_root: Path
    source_episodes: int
    source_frames: int
    output_episodes: int
    output_frames: int


def _as_numpy(value: Any, *, dtype: np.dtype[Any] | None = None) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    array = np.asarray(value)
    return array.astype(dtype, copy=False) if dtype is not None else array


def _scalar(value: Any) -> float:
    array = _as_numpy(value)
    if array.size != 1:
        raise ValueError(f"expected a scalar, got shape {array.shape}")
    return float(array.reshape(-1)[0])


def _name_indices(names: Sequence[str], required: Sequence[str], *, feature: str) -> list[int]:
    if len(names) != len(set(names)):
        raise ValueError(f"{feature} contains duplicate dimension names")
    index = {name: position for position, name in enumerate(names)}
    missing = [name for name in required if name not in index]
    if missing:
        raise ValueError(f"{feature} is missing required dimensions: {', '.join(missing)}")
    return [index[name] for name in required]


def convert_vectors(
    state: Any,
    next_state: Any,
    endpose_action: Any,
    *,
    state_names: Sequence[str],
    action_names: Sequence[str],
) -> ConvertedVectors:
    """Split one legacy frame and derive its next-state joint action."""
    state_array = _as_numpy(state, dtype=np.float32).reshape(-1)
    next_state_array = _as_numpy(next_state, dtype=np.float32).reshape(-1)
    old_action_array = _as_numpy(endpose_action, dtype=np.float32).reshape(-1)

    if state_array.size != len(state_names) or next_state_array.size != len(state_names):
        raise ValueError(
            "observation.state value length does not match its names: "
            f"state={state_array.size}, next_state={next_state_array.size}, "
            f"names={len(state_names)}"
        )
    if old_action_array.size != len(action_names):
        raise ValueError(
            "action value length does not match its names: "
            f"action={old_action_array.size}, names={len(action_names)}"
        )

    state_joint_indices = _name_indices(state_names, JOINT_NAMES, feature="observation.state")
    state_endpose_indices = _name_indices(
        state_names, ENDPOSE_NAMES, feature="observation.state"
    )
    action_endpose_indices = _name_indices(action_names, ENDPOSE_NAMES, feature="action")
    action_gripper_indices = _name_indices(action_names, GRIPPER_NAMES, feature="action")

    observation_state = state_array[state_joint_indices]
    observation_endpose = state_array[state_endpose_indices]
    action_endpose_array = old_action_array[action_endpose_indices]

    next_state_index = {name: position for position, name in enumerate(state_names)}
    next_joint_values = np.asarray(
        [next_state_array[next_state_index[name]] for name in JOINT_ONLY_NAMES],
        dtype=np.float32,
    )
    commanded_grippers = old_action_array[action_gripper_indices]
    joint_action = np.concatenate(
        (
            next_joint_values[:6],
            commanded_grippers[:1],
            next_joint_values[6:],
            commanded_grippers[1:],
        )
    ).astype(np.float32, copy=False)

    converted = ConvertedVectors(
        observation_state=observation_state.astype(np.float32, copy=False),
        observation_endpose=observation_endpose.astype(np.float32, copy=False),
        action=joint_action,
        action_endpose=action_endpose_array.astype(np.float32, copy=False),
    )
    for name, value, expected_shape in (
        ("observation.state", converted.observation_state, (14,)),
        ("observation.state.endpose", converted.observation_endpose, (12,)),
        ("action", converted.action, (14,)),
        ("action.endpose", converted.action_endpose, (12,)),
    ):
        if value.shape != expected_shape:
            raise ValueError(f"{name} has shape {value.shape}, expected {expected_shape}")
        if not np.isfinite(value).all():
            raise ValueError(f"{name} contains NaN or infinity")
    return converted


def build_output_features(source_features: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """Build the requested schema while preserving all source visual features."""
    required = {"observation.state", "action"}
    missing = sorted(required - set(source_features))
    if missing:
        raise ValueError(f"source feature schema is missing: {', '.join(missing)}")

    visual_features: dict[str, dict[str, Any]] = {}
    for key, feature in source_features.items():
        if feature.get("dtype") not in {"image", "video"}:
            continue
        copied = copy.deepcopy(feature)
        copied["shape"] = tuple(copied["shape"])
        # Output videos are re-encoded; LeRobot must probe their new codec info.
        copied.pop("info", None)
        visual_features[key] = copied

    if not visual_features:
        raise ValueError("source dataset has no image or video features")

    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (14,),
            "names": list(JOINT_NAMES),
        },
        "observation.state.endpose": {
            "dtype": "float32",
            "shape": (12,),
            "names": list(ENDPOSE_NAMES),
        },
        "action": {
            "dtype": "float32",
            "shape": (14,),
            "names": list(JOINT_NAMES),
        },
        "action.endpose": {
            "dtype": "float32",
            "shape": (12,),
            "names": list(ENDPOSE_NAMES),
        },
        **visual_features,
    }


def _image_for_writer(value: Any, expected_shape: Sequence[int]) -> np.ndarray:
    image = _as_numpy(value)
    height, width, channels = (int(dimension) for dimension in expected_shape)
    if image.shape == (channels, height, width):
        image = np.moveaxis(image, 0, -1)
    if image.shape != (height, width, channels):
        raise ValueError(
            f"decoded image has shape {image.shape}, expected "
            f"{(height, width, channels)} or {(channels, height, width)}"
        )
    if np.issubdtype(image.dtype, np.floating):
        maximum = float(np.nanmax(image)) if image.size else 0.0
        if maximum <= 1.0 + 1e-6:
            image = image * 255.0
        image = np.rint(np.clip(image, 0.0, 255.0)).astype(np.uint8)
    elif image.dtype != np.uint8:
        image = np.clip(image, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(image)


def _episode_bounds(dataset: Any) -> list[tuple[int, int, int]]:
    metadata = dataset.meta.episodes
    if metadata is None:
        raise ValueError("source dataset has no episode metadata")

    records: list[dict[str, Any]]
    if hasattr(metadata, "to_dict"):
        try:
            records = list(metadata.to_dict(orient="records"))
        except TypeError:
            records = list(metadata)
    else:
        records = list(metadata)

    bounds: list[tuple[int, int, int]] = []
    for expected_episode, record in enumerate(records):
        if not isinstance(record, dict):
            record = dict(record)
        episode_index = int(record["episode_index"])
        start = int(record["dataset_from_index"])
        stop = int(record["dataset_to_index"])
        if episode_index != expected_episode:
            raise ValueError("source episode indices are not contiguous from zero")
        if stop - start < 2:
            raise ValueError(
                f"episode {episode_index} has fewer than two frames and cannot form state[t+1]"
            )
        bounds.append((episode_index, start, stop))
    return bounds


def inspect_source(dataset: Any) -> tuple[list[str], list[str], list[tuple[int, int, int]]]:
    features = dataset.meta.features
    state_feature = features.get("observation.state", {})
    action_feature = features.get("action", {})
    state_names = state_feature.get("names")
    action_names = action_feature.get("names")
    if not isinstance(state_names, list) or not all(isinstance(name, str) for name in state_names):
        raise ValueError("source observation.state must have a string names list")
    if not isinstance(action_names, list) or not all(
        isinstance(name, str) for name in action_names
    ):
        raise ValueError("source action must have a string names list")

    _name_indices(state_names, JOINT_NAMES, feature="observation.state")
    _name_indices(state_names, ENDPOSE_NAMES, feature="observation.state")
    _name_indices(action_names, ENDPOSE_NAMES, feature="action")
    _name_indices(action_names, GRIPPER_NAMES, feature="action")
    return state_names, action_names, _episode_bounds(dataset)


def _write_conversion_info(summary: ConversionSummary) -> None:
    payload = {
        "schema": "dual_pika_piper_joint_eef_v1",
        "source_root": str(summary.source_root),
        "source_episodes": summary.source_episodes,
        "source_frames": summary.source_frames,
        "output_episodes": summary.output_episodes,
        "output_frames": summary.output_frames,
        "joint_action_source": "observation.state[t+1] proxy",
        "gripper_action_source": "legacy action[t]",
        "terminal_frame_policy": "drop the final frame of every episode",
        "source_modified": False,
    }
    (summary.output_root / "CONVERSION_INFO.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def convert_dataset(
    source_root: Path,
    output_root: Path,
    *,
    source_repo_id: str,
    output_repo_id: str,
    vcodec: str,
    image_writer_threads: int,
    dry_run: bool,
) -> ConversionSummary | None:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    source_root = source_root.expanduser().resolve(strict=True)
    output_root = output_root.expanduser().resolve(strict=False)
    if not source_root.is_dir():
        raise ValueError(f"source root is not a directory: {source_root}")
    if source_root == output_root:
        raise ValueError("source and output dataset roots must be different")
    if output_root.exists():
        raise FileExistsError(f"output path already exists: {output_root}")

    source = LeRobotDataset(source_repo_id, root=source_root)
    state_names, action_names, episode_bounds = inspect_source(source)
    output_features = build_output_features(source.meta.features)
    source_frames = int(source.meta.total_frames)
    expected_output_frames = sum(stop - start - 1 for _, start, stop in episode_bounds)

    print(f"source:              {source_root}")
    print(f"episodes:            {len(episode_bounds)}")
    print(f"source frames:       {source_frames}")
    print(f"converted frames:    {expected_output_frames}")
    print(f"terminal frames cut: {len(episode_bounds)}")
    print("joint action source: observation.state[t+1] proxy")
    if dry_run:
        print("dry-run passed; no output directory was created")
        return None

    output_root.parent.mkdir(parents=True, exist_ok=True)
    partial_root = output_root.with_name(f".{output_root.name}.partial-{uuid.uuid4().hex}")
    target = None
    try:
        target = LeRobotDataset.create(
            repo_id=output_repo_id,
            fps=int(source.meta.fps),
            features=output_features,
            root=partial_root,
            robot_type=source.meta.robot_type,
            use_videos=True,
            image_writer_threads=image_writer_threads,
            vcodec=vcodec,
        )
        visual_keys = [
            key
            for key, feature in output_features.items()
            if feature["dtype"] in {"image", "video"}
        ]

        written_frames = 0
        for episode_index, start, stop in episode_bounds:
            for source_index in range(start, stop - 1):
                sample = source[source_index]
                next_state = source.hf_dataset[source_index + 1]["observation.state"]
                vectors = convert_vectors(
                    sample["observation.state"],
                    next_state,
                    sample["action"],
                    state_names=state_names,
                    action_names=action_names,
                )
                frame: dict[str, Any] = {
                    "observation.state": vectors.observation_state,
                    "observation.state.endpose": vectors.observation_endpose,
                    "action": vectors.action,
                    "action.endpose": vectors.action_endpose,
                    "timestamp": _scalar(sample["timestamp"]),
                    "task": str(sample["task"]),
                }
                for key in visual_keys:
                    frame[key] = _image_for_writer(sample[key], output_features[key]["shape"])
                target.add_frame(frame)
                written_frames += 1
            target.save_episode()
            print(
                f"episode {episode_index}: saved {stop - start - 1} frames "
                f"({episode_index + 1}/{len(episode_bounds)})"
            )

        target.finalize()
        target = None
        if written_frames != expected_output_frames:
            raise RuntimeError(
                f"converted frame mismatch: wrote {written_frames}, "
                f"expected {expected_output_frames}"
            )
        partial_root.rename(output_root)
    except BaseException:
        if target is not None:
            try:
                target.finalize()
            except Exception:
                pass
        if partial_root.exists():
            print(f"partial output was preserved for diagnosis: {partial_root}", file=sys.stderr)
        raise

    summary = ConversionSummary(
        source_root=source_root,
        output_root=output_root,
        source_episodes=len(episode_bounds),
        source_frames=source_frames,
        output_episodes=len(episode_bounds),
        output_frames=expected_output_frames,
    )
    _write_conversion_info(summary)

    # Re-open the finalized output through the same loader used for training.
    converted = LeRobotDataset(output_repo_id, root=output_root)
    if int(converted.meta.total_episodes) != summary.output_episodes:
        raise RuntimeError("final output episode count does not match conversion plan")
    if int(converted.meta.total_frames) != summary.output_frames:
        raise RuntimeError("final output frame count does not match conversion plan")
    expected_shapes = {
        "observation.state": [14],
        "observation.state.endpose": [12],
        "action": [14],
        "action.endpose": [12],
    }
    for key, expected_shape in expected_shapes.items():
        actual_shape = list(converted.meta.features[key]["shape"])
        if actual_shape != expected_shape:
            raise RuntimeError(f"{key} has shape {actual_shape}, expected {expected_shape}")

    print(f"conversion complete: {output_root}")
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert the legacy 26-state/14-EEF-action dual Piper dataset to "
            "separate joint and endpose features"
        )
    )
    parser.add_argument("--source", type=Path, required=True, help="Existing legacy dataset root")
    parser.add_argument("--output", type=Path, required=True, help="New converted dataset root")
    parser.add_argument(
        "--source-repo-id",
        help="LeRobot source repo id (default: local/<source directory name>)",
    )
    parser.add_argument(
        "--output-repo-id",
        help="LeRobot output repo id (default: local/<output directory name>)",
    )
    parser.add_argument(
        "--vcodec",
        choices=("h264", "hevc", "libsvtav1"),
        default="h264",
        help="Codec used when re-encoding the converted videos (default: h264)",
    )
    parser.add_argument(
        "--image-writer-threads",
        type=int,
        default=8,
        help="Temporary image writer threads used by LeRobot (default: 8)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate source schema and print the conversion plan without writing output",
    )
    args = parser.parse_args(argv)
    if args.image_writer_threads < 0:
        parser.error("--image-writer-threads must be non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    source_repo_id = args.source_repo_id or f"local/{args.source.expanduser().name}"
    output_repo_id = args.output_repo_id or f"local/{args.output.expanduser().name}"
    try:
        convert_dataset(
            args.source,
            args.output,
            source_repo_id=source_repo_id,
            output_repo_id=output_repo_id,
            vcodec=args.vcodec,
            image_writer_threads=args.image_writer_threads,
            dry_run=args.dry_run,
        )
    except Exception as exc:
        print(f"conversion failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
